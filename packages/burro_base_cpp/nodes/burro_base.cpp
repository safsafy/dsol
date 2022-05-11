

#include <GeometryMsgs/Twist.h>
#include <StdMsgs/Bool.h>
#include <StdMsgs/Float64.h>
#include <StdSrvs/Trigger.h>
#include <StdSrvs/TriggerResponse.h>

#include <burro_base/BurroBaseStatus.h>
#include <burro_base/Encoder.h>
#include <burro_msgs/SetString.h>

#include <roboteq/roboteq_multi_device.h>
#include <roboteq/roboteq_serial.h>


class BurroControl : public RosNode
{

    // Device
    RoboteqMultiDevice drivers_;

    // ROS resources
    std::map<cost char*, ros::ServiceServer> services_
    std::map<const char*, ros::Publisher> publishers_;
    std::map<const char*, ros::Subscribers> subscribers_;

    // Configuration
    bool estop_auto_reset_;


    void OnInit()
    {
        drivers_.AddDevice<RoboteqSerialDevice>("front");
        drivers_.AddDevice<RoboteqSerialDevice>("rear");

        pnh().param(config_, "config");

        publishers_["status"] = pnh().advertise<burro_base::BurroBaseStatus>("status", 1);
        publishers_["encoders"] = pnh().advertise<burro_base::Encoders>("encoders", 1);
        subscribers_["cmd_vel"] = pnh().subscribe("cmd_vel", 5, &BurroControl::CmdVelCallback, this);

        services_["command"] = pnh().advertiseService("command", &BurroControl::CommandService, this);

        // Diagnostics
        //diagnostics_.add(subscribers_["cmd_vel"]);
        //diagnostics_.add(subscribers_["status"]);
        //diagnostics_.add("Status", &BurroControl::StatusDiagnostic, this);
    }

    void NodeDiagnostic(DiagnosticStatusWrapper& stat) const
    {
        if (!driver_.connected())
        {
            stat.summary(2, "#E601 Device not connected")
        }
    }


    void StatusDiagnostic(DisgnosticStatusWrapper& stat)
    {
        stat.add("Stopped", Safestop());
        stat.add("E-stop", Estop());

        StatWrapper summary;

        if (Estop())
        {
            summary.AddMessage("#E603 Estop Active", 2);
        }

        if (Safestop())
        {
            summary.AddMessage("Stopped", 1);
        }
        stat.summary(summary.level(), summary.message());
    }

    void CommandService(SetStringConstPtr& srv)
    {
        SetStringResponse response;
        response.success = true;
        auto response = driver_.SendCommand(srv->data);

        response.message = response;
        return response;
    }

    def reset_estop_service(self, srv):
        """reset_estop_service: service callback to reset the e-stop."""
        return TriggerResponse(*self.reset_estop())

    def reset_estop(self):
        """reset_estop: estop reset handler."""
        self.stop_motors()
        success = [d.reset_estop() for d in self.driver]

        msg = 'EStop reset success={}'.format(success)
        return np.all(success), msg

    def reset_safe_stop(self):
        """reset_safestop: stop motors so that reset can be completed."""
        self.stop_motors()
        return True, 'motor speeds set to 0.'

    def reset_safe_stop_service(self, srv):
        """reset_safe_stop_service: service callback to reset the safe stop."""
        return TriggerResponse(*self.reset_safe_stop())

    def set_lights_enabled(self, enabled):
        """turn_lights_on: enable digital output to turn on the lights."""
        success = []
        for d, v in zip(self.driver, enabled):
            if v:
                success.append(d.enable_digital_output(1))
            else:
                success.append(d.disable_digital_output(1))

        msg = 'Setting lights {}. Success={}'.format(enabled, success)
        return np.all(success), msg

    def turn_lights_on_service(self, srv):
        """turn_lights_on_service: service callback to turn the lights on."""
        return TriggerResponse(*self.set_lights_enabled([True, True]))

    def turn_lights_off_service(self, srv):
        """turn_lights_on_service: service callback to turn the lights off."""
        return TriggerResponse(*self.set_lights_enabled([False, False]))

    def enable_callback(self, state):
        """enable_callback: message callback."""
        self.__enabled = state.data

    def estop(self):
        """estop: estop driver message parser."""
        return self.base_status.is_emergency_stopped()

    def safestop(self):
        """estop: estop driver message parser."""
        return self.base_status.is_stopped()

    def no_cmd_vel_received(self, t):
        """no_cmd_vel_received: report if cmd_vel not received for period t."""
        return time.time() - self.cmd_vel_last_received > t

    def enabled(self):
        """enabled: report if enabled and estop and safestop not active."""
        return self.__enabled and not self.estop() and not self.safestop()

    def cmd_vel_callback(self, msg):
        """cmd_vel_callback: message callback."""
        self.curr_cmd_vel = msg
        if (msg.linear.x != 0 or msg.angular.z != 0):
            self.cmd_vel_last_received = time.time()

    def set_velocity_gains(self):
        """set_velocity_gains: set velocity gains."""
        status = True
        for d in self.driver:
            ret = d.set_PID([self.KP, self.KI, self.KD])
            status = status & ret
        return status

    def set_io(self):
        """ Configure motor drivers """
        status = True
        for d in self.driver:
            # Disable pulse input
            status &= d._set("^PINA", 1, 0)
            status &= d._set("^PINA", 2, 0)
            status &= d._set("^PMOD", 1, 0)
            status &= d._set("^PMOD", 2, 0)
            # Set deceleration rate
            status &= d._set("^MDEC", 1, 40000)
            status &= d._set("^MDEC", 2, 40000)
        return status

    def set_max_rpm(self):
        """set_max_rpm: set maximum motor speed in rpm."""
        status = True
        for d in self.driver:
            ret = d.set_max_motor_speed_rpm([self.max_rpm] * 2)
            status = status & ret
        return status

    def read_encoders(self):
        """read_encoders: read encoder values."""
        return [d.get_encoders()[1] for d in self.driver]

    def update_status(self):
        """update_status: update the drive safety and monitoring readings."""
        self.base_status.header.stamp = rospy.Time.now()
        for driver, status in zip(self.driver, self.base_status.controllers):
            read_errors = []

            flag, volt = driver.get_battery_voltage()
            if flag:
                curr_step = volt - status.voltage
                status.voltage += self.ka_status * curr_step
            else:
                read_errors.append("voltage")

            flag, curs = driver.get_motor_currents()
            if flag:
                current = np.array(status.current)
                curr_step = np.array(curs) - current
                status.current = (current + self.ka_status * curr_step).tolist()
            else:
                read_errors.append("current")

            flag, temp = driver.get_temperature()
            if flag:
                curr_step = temp - status.temperature
                status.temperature += self.ka_status * curr_step
            else:
                read_errors.append("temperature")

            flag, dout = driver.get_all_digital_outputs()
            if flag:
                status.led_status = dout[0]
            else:
                read_errors.append("led")

            if len(read_errors):
                rospy.logwarn("Driver({}): Unable to read [{}]"
                              .format(status.id, ";".join(read_errors)))

    def update_faults(self, event=None):
        last_safestop = self.safestop()
        for driver, status in zip(self.driver, self.base_status.controllers):
            read_errors = []
            flag, driver_faults, driver_status, motor_status = driver.get_and_parse_flags()
            if flag:
                status.error = driver_faults[0]
                status.driver_status = driver_status[0]
                status.motor1_status = motor_status[0][0]
                status.motor2_status = motor_status[1][0]
            else:
                read_errors.append("faults")

            flag, inputs = driver.get_inputs()
            if flag:
                for key in self.input_name:
                    status.set_signal(key,
                                      driver.inps[key].inp_flag,
                                      driver.inps[key].inp_status)
            else:
                read_errors.append("inputs")

            if len(read_errors):
                rospy.logwarn("Driver({}): Unable to read [{}]"
                              .format(status.id, ";".join(read_errors)))

        if self.estop_auto_reset:
            # Do we reset ESTOP? Detect rising edge
            if self.estop() and not self.safestop() and last_safestop:
                # Stop button has been pulled
                success, msg = self.reset_estop()
                rospy.loginfo("Auto " + msg)

    def publish_current(self, event):
        current = np.array(self.base_status.get_current())
        left_current = sum(current[self.left_id])
        right_current = sum(current[self.right_id])

        left_avg = self.moving_average_left.update(left_current)
        right_avg = self.moving_average_right.update(right_current)

        self.avg_curr_left_pub.publish(left_avg)
        self.avg_curr_right_pub.publish(right_avg)
        self.max_curr_left_pub.publish(max(self.moving_average_left.window))
        self.max_curr_right_pub.publish(max(self.moving_average_right.window))

    def publish_status(self):
        """publish_status: publish the burro_base/status message."""
        self.status_pub.publish(self.base_status)
        self.battery_pub.publish(self.base_status.get_voltage())
        self.temperature_pub.publish(self.base_status.get_temperature())

    def publish_encoders(self):
        enc = Encoder()
        enc.stamp = rospy.Time.now()

        encoders = self.read_encoders()
        for c, e in zip(self.base_status.controllers, encoders):
            c.encoder = e

        ticks = []
        # Chain multiple lists coming from read_encoders
        map(ticks.extend, encoders)
        enc.motor_id = self.base_status.get_motor_id()
        enc.encoder = list(ticks)
        self.encoders_pub.publish(enc)

    def _send_vel(self, tick_per_sec):
        """_send_vel: send the ticks/sec message to the drivers."""
        if np.all(self.port_status):
            # go from ticks_per_sec to revolutions per minute
            scale = 60.0 / (4 * self.encoder_ppr)
            for i in xrange(self.num_driver):
                vel = [int(tick_per_sec[i * 2] * scale),
                       int(tick_per_sec[i * 2 + 1] * scale)]
                flag = self.driver[i].set_motor_speed_rpm(vel)
                if not flag:
                    rospy.logwarn('[Drive {}] Failed to send the speed'.
                                  format(i))
                # For driver debugging
                self.base_cmd = RoboclawCommand(type='set_rpm',
                                                driver_id=i,
                                                values=vel)

    def stop_motors(self):
        """
        stop_motors: send 0 velocity.

        Let the drivers handle deceleration.
        """
        self.curr_cmd_vel = Twist()  # clean up the cmd_vel
        self._send_vel(np.zeros(2 * self.num_driver))

    def on_shutdown(self):
        """shutdown: shutdown method."""
        self.stop_motors()
        [(t.shutdown(), t.join()) for t in self.timers]
        for d in self.driver:
            d.Close()

    def drive_thread(self, event):
        """drive_thread: thread handling spinning of the motors."""
        if not self.enabled():
            self.stop_motors()
            return

        if self.no_cmd_vel_received(self.drive_threshold):
            self.stop_motors()
            return

        curr_v = np.clip(self.curr_cmd_vel.linear.x, self.min_v, self.max_v)
        curr_w = np.clip(self.curr_cmd_vel.angular.z, -self.max_w, self.max_w)

        angular = 0.5 * curr_w * self.base_width * self.base_width_multiplier

        # left_vel = v - w * (separation/2)
        # meters/sec
        self.cmd_side_vel[0] = curr_v - angular

        # right_vel = v + w * (separation/2)
        # meters/sec
        self.cmd_side_vel[1] = curr_v + angular

        self.cmd_wheel_vel[self.left_id] = self.cmd_side_vel[0]
        self.cmd_wheel_vel[self.right_id] = self.cmd_side_vel[1]

        # convert to ticks/sec
        self.cmd_wheel_vel *= self.ticks_per_meter
        self.cmd_wheel_vel *= self.direction

        self._send_vel(self.cmd_wheel_vel)

    def odometry_thread(self, event):
        """odometry_thread: thread handling updating/publishing of odometry."""
        self.publish_encoders()

    def status_thread(self, event):
        """status_thread: thread handling updating/publishing of status."""
        self.update_status()
        self.publish_status()

    def connect(self):
        """connet: motor driver serial communication method."""
        r = rospy.Rate(0.5)
        while not rospy.is_shutdown() and not self.connected:
            self.port_status = [d.Open() for d in self.driver]
            self.connected = np.all(self.port_status)

            if not self.connected:
                index = [i for i, p in enumerate(self.port_status) if not p]
                rospy.logerr('Devices {} not available. will try again'
                             .format(index))

            elif not self.set_velocity_gains():
                rospy.logerr("Could not set velocity gains")
                self.connected = False

            elif not self.set_io():
                rospy.logerr("Could not set IO config")
                self.connected = False

            if not self.connected:
                [d.Close() for d in self.driver]
                self._tick()  # Keeps diagnostics going
                r.sleep()

        if self.connected:
            rospy.loginfo('Driver connection established.')
            self.timers = [
                rospy.Timer(rospy.Duration(1.0 / self.odometry_freq), self.odometry_thread),
                rospy.Timer(rospy.Duration(1.0 / 4.0), self.update_faults),
                rospy.Timer(rospy.Duration(1.0 / self.status_freq), self.status_thread),
                rospy.Timer(rospy.Duration(1.0 / self.drive_freq), self.drive_thread),
                rospy.Timer(rospy.Duration(1.0 / self.status_freq), self.publish_current)
            ]
