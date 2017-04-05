/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef COMMON_INTERFACES_INTERFACE_RX_H__
#define COMMON_INTERFACES_INTERFACE_RX_H__

#include "shm_comm/shm_channel.h"

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/Logger.hpp"
#include <rtt_rosclock/rtt_rosclock.h>

using namespace RTT;

template <class Container>
class InterfaceRx: public RTT::TaskContext {
public:
    explicit InterfaceRx(const std::string& name)
        : TaskContext(name, PreOperational)
        , shm_name_("TODO")
        , buf_prev_(NULL)
        , port_msg_out_("msg_OUTPORT", false)
        , no_data_out_("no_data_OUTPORT")
        , event_(false)
        , period_min_(0.0)
        , period_avg_(0.0)
        , period_max_(0.0)
    {
        this->ports()->addPort(port_msg_out_);
        this->ports()->addPort(no_data_out_);

        this->addOperation("getDiag", &InterfaceRx::getDiag, this, RTT::ClientThread);

        addProperty("event", event_);
        addProperty("period_min", period_min_);
        addProperty("period_avg", period_avg_);
        addProperty("period_max", period_max_);

        addProperty("channel_name", param_channel_name_);
    }

    // this method in not RT-safe
    std::string getDiag() {
        std::stringstream ss;
        if (diag_buf_valid_) {
            //ros::message_operations::Printer<Container >::stream(ss, "", diag_buf_);
            ss << "<data ok>";
        }
        else {
            ss << "<no data>";

            RTT::os::TimeService::Seconds last_recv_sec = RTT::nsecs_to_Seconds(last_recv_time_);
            RTT::os::TimeService::Seconds now_sec = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs());
            RTT::Seconds interval = now_sec - last_recv_sec;
            ss << ", <for: " << interval << "s>";
        }
        return ss.str();
    }

    bool configureHook() {
        Logger::In in("InterfaceRx::configureHook");

        if (param_channel_name_.empty()) {
            Logger::log() << Logger::Error << "parameter \'channel_name\' is empty" << Logger::endl;
            return false;
        }

        if (event_) {
            if (period_min_ == 0.0) {
                Logger::log() << Logger::Error << "parameter \'period_min\' is not set (0.0)" << Logger::endl;
                return false;
            }
            if (period_avg_ == 0.0) {
                Logger::log() << Logger::Error << "parameter \'period_avg\' is not set (0.0)" << Logger::endl;
                return false;
            }
            if (period_max_ == 0.0) {
                Logger::log() << Logger::Error << "parameter \'period_max\' is not set (0.0)" << Logger::endl;
                return false;
            }

            if (period_min_ >= period_avg_) {
                Logger::log() << Logger::Error << "parameter \'period_min\' should be < than \'period_avg\': " << period_min_ << ", " << period_avg_ << Logger::endl;
                return false;
            }
            if (period_avg_ >= period_max_) {
                Logger::log() << Logger::Error << "parameter \'period_avg_\' should be < than \'period_max_\': " << period_avg_ << ", " << period_max_ << Logger::endl;
                return false;
            }
        }

        Logger::log() << Logger::Info << "parameter channel_name is set to: \'" << param_channel_name_ << "\'" << Logger::endl;
        Logger::log() << Logger::Info << "parameter event is set to: \'" << (event_?"true":"false") << Logger::endl;
        Logger::log() << Logger::Info << "parameter \'period_min\' is set to: " << period_min_ << Logger::endl;
        Logger::log() << Logger::Info << "parameter \'period_avg\' is set to: " << period_avg_ << Logger::endl;
        Logger::log() << Logger::Info << "parameter \'period_max\' is set to: " << period_max_ << Logger::endl;

        shm_name_ = param_channel_name_;

        bool create_channel = false;

        int result = shm_connect_reader(shm_name_.c_str(), &re_);
        if (result == SHM_INVAL) {
            Logger::log() << Logger::Error << "shm_connect_reader: invalid parameters" << Logger::endl;
            return false;
        }
        else if (result == SHM_FATAL) {
            Logger::log() << Logger::Error << "shm_connect_reader: memory error" << Logger::endl;
            return false;
        }
        else if (result == SHM_NO_CHANNEL) {
            Logger::log() << Logger::Warning << "shm_connect_reader: could not open shm object, trying to initialize the channel..." << Logger::endl;
            create_channel = true;
        }
        else if (result == SHM_CHANNEL_INCONSISTENT) {
            Logger::log() << Logger::Warning << "shm_connect_reader: shm channel is inconsistent, trying to initialize the channel..." << Logger::endl;
            create_channel = true;
        }
        else if (result == SHM_ERR_INIT) {
            Logger::log() << Logger::Error << "shm_connect_reader: could not initialize channel" << Logger::endl;
            return false;
        }
        else if (result == SHM_ERR_CREATE) {
            Logger::log() << Logger::Warning << "shm_connect_reader: could not create reader" << Logger::endl;
            create_channel = true;
        }

        if (!create_channel) {
            void *pbuf = NULL;
            result = shm_reader_buffer_get(re_, &pbuf);
            if (result < 0) {
                Logger::log() << Logger::Warning << "shm_reader_buffer_get: error: " << result << Logger::endl;
                create_channel = true;
            }
        }

        if (create_channel) {
            result = shm_create_channel(shm_name_.c_str(), sizeof(Container), 1, true);
            if (result != 0) {
                Logger::log() << Logger::Error << "create_shm_object: error: " << result << "   errno: " << errno << Logger::endl;
                return false;
            }

            result = shm_connect_reader(shm_name_.c_str(), &re_);
            if (result != 0) {
                Logger::log() << Logger::Error << "shm_connect_reader: error: " << result << Logger::endl;
                return false;
            }
        }

        mTriggerOnStart = false;

        return true;
    }

    void cleanupHook() {
        shm_release_reader(re_);
    }

    void stopHook() {
        if (getTaskState() == TaskContext::Exception || getTaskState() == TaskContext::RunTimeError) {
            recover();
        }
    }

    bool startHook() {
        void *pbuf = NULL;

        int result = 0;
        result = shm_reader_buffer_get(re_, &pbuf);

        if (result < 0) {
            Logger::log() << Logger::Error << "shm_reader_buffer_get: error: " << result << Logger::endl;
            return false;
        }

        buf_prev_ = reinterpret_cast<Container*>( pbuf );

        diag_buf_valid_ = false;
        trigger();

        last_read_successful_ = false;

        return true;
    }

    void exceptionHook() {
        recover();
    }

    void updateHook() {

        // get update time
        ros::Time update_time = rtt_rosclock::host_now();
        timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);

        void *pbuf = NULL;
        Container *buf = NULL;

        if (event_) {
            double timeout_s;
            if (last_read_successful_) {
                timeout_s = period_max_;
            }
            else {
                timeout_s = period_avg_;
            }

            int timeout_sec = (int)timeout_s;
            int timeout_nsec = (int)((timeout_s - (double)timeout_sec) * 1000000000.0);

            ts.tv_sec += timeout_sec;
            ts.tv_nsec += timeout_nsec;
            if (ts.tv_nsec >= 1000000000) {
                ts.tv_nsec -= 1000000000;
                ++ts.tv_sec;
            }

            int read_status = shm_reader_buffer_timedwait(re_, &ts, &pbuf);

            ros::Time read_time = rtt_rosclock::host_now();
            double read_interval = (read_time - update_time).toSec();

            if (read_status == SHM_TIMEOUT) {
                diag_buf_valid_ = false;
                last_read_successful_ = false;
                no_data_out_.write(true);
                // do not wait
            }
            else if (read_status == 0 && ((buf = reinterpret_cast<Container*>( pbuf )) != buf_prev_)) {
                diag_buf_valid_ = true;
                last_read_successful_ = true;
                // save the pointer of buffer
                buf_prev_ = buf;
                port_msg_out_.write(*buf);
                if (read_interval < period_min_) {
                    usleep( int((period_min_ - read_interval)*1000000.0) );
                }
            }
            else if (read_status > 0) {
                diag_buf_valid_ = false;
                last_read_successful_ = false;
                no_data_out_.write(true);
                if (read_interval < period_avg_) {
                    usleep( int((period_avg_ - read_interval)*1000000.0) );
                }
            }
            else {
                diag_buf_valid_ = false;
                Logger::log() << Logger::Error << getName() << " shm_reader_buffer_timedwait status: " << read_status << Logger::endl;
                error();
                return;
            }
        }
        else {
            double timeout_s = 1.0;

            int timeout_sec = (int)timeout_s;
            int timeout_nsec = (int)((timeout_s - (double)timeout_sec) * 1000000000.0);

            ts.tv_sec += timeout_sec;
            ts.tv_nsec += timeout_nsec;
            if (ts.tv_nsec >= 1000000000) {
                ts.tv_nsec -= 1000000000;
                ++ts.tv_sec;
            }

            int read_status = shm_reader_buffer_timedwait(re_, &ts, &pbuf);

            if (read_status == SHM_TIMEOUT) {
                diag_buf_valid_ = false;
            }
            else if (read_status == 0 && ((buf = reinterpret_cast<Container*>( pbuf )) != buf_prev_)) {
                diag_buf_valid_ = true;
                // save the pointer of buffer
                buf_prev_ = buf;
                port_msg_out_.write(*buf);
            }
            else if (read_status > 0) {
                diag_buf_valid_ = false;
            }
            else {
                diag_buf_valid_ = false;
                Logger::log() << Logger::Error << getName() << " shm_reader_buffer_timedwait status: " << read_status << Logger::endl;
                error();
                return;
            }

        }

        trigger();
    }

private:

    // properties
    std::string param_channel_name_;

    bool event_;
    double period_min_;
    double period_avg_;
    double period_max_;

    std::string shm_name_;

    shm_reader_t* re_;
    Container *buf_prev_;
    bool last_read_successful_;

    RTT::OutputPort<Container > port_msg_out_;

    RTT::OutputPort<bool > no_data_out_;

    bool diag_buf_valid_;

    RTT::os::TimeService::nsecs last_recv_time_;

    ros::Time last_update_time_;
};

#endif  // COMMON_INTERFACES_INTERFACE_RX_H__

