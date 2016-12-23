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

#ifndef __COMMON_INTERFACES_DATA_CONVERSION_H__
#define __COMMON_INTERFACES_DATA_CONVERSION_H__

#include <cstring>
#include <vector>
#include <string>

#include "common_interfaces/interface_ports.h"


namespace interface_ports {

template <typename rosT, typename oroT >
class InputPortConv : public InputPortInterface<rosT > {
public:
    explicit InputPortConv(RTT::TaskContext *tc, const std::string &port_name)
        : port_(new RTT::InputPort<oroT >(port_name + "_INPORT"))
        , tc_(tc)
    {
        tc->ports()->addLocalPort(*port_);
    }

    virtual bool read(rosT &data_ros) {
        oroT data_oro;
        if (port_->read(data_oro, false) != RTT::NewData) {
            return false;
        }
        convert(data_oro, data_ros);
        return true;
    }

    virtual void convert(const oroT& data_oro, rosT& data_ros) const = 0;

    bool removeUnconnectedPorts() {
        if (!port_->connected()) {
            tc_->ports()->removeLocalPort( port_->getName() );
            port_.reset();
            return true;
        }
        return false;
    }

protected:

    boost::shared_ptr<RTT::InputPort<oroT > > port_;
    RTT::TaskContext* tc_;
};

template <typename rosT, typename oroT >
class OutputPortConv : public OutputPortInterface<rosT > {
public:
    explicit OutputPortConv(RTT::TaskContext *tc, const std::string &port_name)
        : port_(new RTT::OutputPort<oroT >(port_name + "_OUTPORT", false))
        , tc_(tc)
    {
        tc->ports()->addLocalPort(*port_);
    }

    virtual bool write(const rosT &data_ros) {
        oroT data_oro;
        convert(data_ros, data_oro);
        port_->write(data_oro);
        return true;
    }

    virtual void convert(const rosT& data_ros, oroT& data_oro) const = 0;

    bool removeUnconnectedPorts() {
        if (!port_->connected()) {
            tc_->ports()->removeLocalPort( port_->getName() );
            port_.reset();
            return true;
        }
        return false;
    }

protected:

    boost::shared_ptr<RTT::OutputPort<oroT > > port_;
    RTT::TaskContext* tc_;
};

};  // namespace interface_ports

// data conversion stuff
// this trick is from http://stackoverflow.com/questions/13842468/comma-in-c-c-macro
template<typename T> struct argument_type;
template<typename T, typename U> struct argument_type<T(U)> { typedef U type; };
// e.g.: #define FOO(t,name) argument_type<void(t)>::type name

#define REGISTER_DATA_CONVERSION(ROS_MSG_NAMESPACE, ROS_MSG_MSG, ROS_MSG_FIELD, ROS_T, ORO_T, ROS_ORO_CODE, ORO_ROS_CODE) class CONV_TO_ORO_##ROS_MSG_NAMESPACE##ROS_MSG_MSG##ROS_MSG_FIELD :\
public interface_ports::OutputPortConv<argument_type<void(ROS_T)>::type, argument_type<void(ORO_T)>::type > {\
public:\
    typedef argument_type<void(ROS_T)>::type Container_;\
    CONV_TO_ORO_##ROS_MSG_NAMESPACE##ROS_MSG_MSG##ROS_MSG_FIELD(RTT::TaskContext *tc, const std::string &port_name)\
      : interface_ports::OutputPortConv<argument_type<void(ROS_T)>::type, argument_type<void(ORO_T)>::type >(tc, port_name) {}\
    virtual void convert(const argument_type<void(ROS_T)>::type& ros, argument_type<void(ORO_T)>::type& oro) const {ROS_ORO_CODE}\
};\
REGISTER_OutputPortInterface(CONV_TO_ORO_##ROS_MSG_NAMESPACE##ROS_MSG_MSG##ROS_MSG_FIELD, #ROS_MSG_NAMESPACE"_"#ROS_MSG_MSG"_"#ROS_MSG_FIELD);\
class CONV_TO_ROS_##ROS_MSG_NAMESPACE##ROS_MSG_MSG##ROS_MSG_FIELD : public interface_ports::InputPortConv<argument_type<void(ROS_T)>::type, argument_type<void(ORO_T)>::type > {\
public:\
    typedef argument_type<void(ROS_T)>::type Container_;\
    CONV_TO_ROS_##ROS_MSG_NAMESPACE##ROS_MSG_MSG##ROS_MSG_FIELD(RTT::TaskContext *tc, const std::string &port_name)\
      : interface_ports::InputPortConv<argument_type<void(ROS_T)>::type, argument_type<void(ORO_T)>::type >(tc, port_name) {}\
    virtual void convert(const argument_type<void(ORO_T)>::type& oro, argument_type<void(ROS_T)>::type& ros) const {ORO_ROS_CODE}\
};\
REGISTER_InputPortInterface(CONV_TO_ROS_##ROS_MSG_NAMESPACE##ROS_MSG_MSG##ROS_MSG_FIELD, #ROS_MSG_NAMESPACE"_"#ROS_MSG_MSG"_"#ROS_MSG_FIELD);


#endif  // __COMMON_INTERFACES_DATA_CONVERSION_H__

