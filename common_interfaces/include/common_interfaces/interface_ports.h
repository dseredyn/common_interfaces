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

#ifndef __COMMON_INTERFACES_INTERFACE_PORTS_H__
#define __COMMON_INTERFACES_INTERFACE_PORTS_H__

#include <cstring>
#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include "rtt/RTT.hpp"

namespace interface_ports {


template <typename T >
class InputPortInterface {
public:
    virtual bool read(T &data) = 0;
};

template <typename T >
class OutputPortInterface {
public:
    virtual bool write(const T &data) = 0;
};

template <typename T >
class InputPort : public InputPortInterface<T > {
public:
    explicit InputPort(RTT::TaskContext *tc, const std::string &port_name) :
        port_(port_name + "_INPORT")
    {
        tc->ports()->addPort(port_);
    }

    virtual bool read(T &data) {
        return (port_.read(data) == RTT::NewData);
    }

protected:

    RTT::InputPort<T > port_;
};

template <typename T >
class OutputPort : public OutputPortInterface<T > {
public:
    explicit OutputPort(RTT::TaskContext *tc, const std::string &port_name) :
        port_(port_name + "_OUTPORT", true)
    {
        tc->ports()->addPort(port_);
    }

    virtual bool write(const T &data) {
        port_.write(data);
        return true;
    }

protected:

    RTT::OutputPort<T > port_;
};

template <typename rosT, typename oroT >
class InputPortConv : public InputPortInterface<rosT > {
public:
    explicit InputPortConv(RTT::TaskContext *tc, const std::string &port_name) :
        port_(port_name + "_INPORT")
    {
        tc->ports()->addPort(port_);
    }

    virtual bool read(rosT &data_ros) {
        oroT data_oro;
        if (port_.read(data_oro) != RTT::NewData) {
            return false;
        }
        convert(data_oro, data_ros);
        return true;
    }

    virtual void convert(const oroT& data_oro, rosT& data_ros) const = 0;

protected:

    RTT::InputPort<oroT > port_;
};

template <typename rosT, typename oroT >
class OutputPortConv : public OutputPortInterface<rosT > {
public:
    explicit OutputPortConv(RTT::TaskContext *tc, const std::string &port_name) :
        port_(port_name + "_OUTPORT")
    {
        tc->ports()->addPort(port_);
    }

    virtual bool write(const rosT &data_ros) {
        oroT data_oro;
        convert(data_ros, data_oro);
        port_.write(data_oro);
        return true;
    }

    virtual void convert(const rosT& data_ros, oroT& data_oro) const = 0;

protected:

    RTT::OutputPort<oroT > port_;
};

template <typename T >
class InputPortInterfaceFactory
{
private:
    std::map<string, InputPortInterface<T >* (*)(RTT::TaskContext *tc, const std::string& prefix) > factoryFunctionRegistry;

    InputPortInterfaceFactory() {}

public:
    boost::shared_ptr<InputPortInterface<T > > Create(string name, RTT::TaskContext *tc, const std::string& prefix)
    {
        InputPortInterface<T > * instance = NULL;

        // find name in the registry and call factory method.
        typename std::map<string, InputPortInterface<T >* (*)(RTT::TaskContext *tc, const std::string& prefix) >::const_iterator it = factoryFunctionRegistry.find(name);
        if(it != factoryFunctionRegistry.end())
            instance = it->second(tc, prefix);

        // wrap instance in a shared ptr and return
        if(instance != NULL)
            return boost::shared_ptr<InputPortInterface<T > >(instance);
        else
            return boost::shared_ptr<InputPortInterface<T > >();
    }

    void RegisterFactoryFunction(string name, InputPortInterface<T >* (*classFactoryFunction)(RTT::TaskContext *tc, const std::string& prefix) )
    {
        // register the class factory function
        factoryFunctionRegistry[name] = classFactoryFunction;
    }

    static InputPortInterfaceFactory* Instance()
    {
        static InputPortInterfaceFactory factory;
        return &factory;
    }
};

template<class T>
InputPortInterface<typename T::Container_ >* InputPortInterfaceFactoryFunction(RTT::TaskContext *tc, const std::string& prefix) {
    return new T(tc, prefix);
}

template<class T>
class InputPortInterfaceRegistrar {
public:
    InputPortInterfaceRegistrar(const std::string& name)
    {
        // register the class factory function 
        InputPortInterfaceFactory<typename T::Container_ >::Instance()->RegisterFactoryFunction(name, InputPortInterfaceFactoryFunction<T >);
    }
};


// TODO: factory for OutputPortInterface

template <typename T >
class OutputPortInterfaceFactory
{
private:
    std::map<string, OutputPortInterface<T >* (*)(RTT::TaskContext *tc, const std::string& prefix) > factoryFunctionRegistry;

    OutputPortInterfaceFactory() {}

public:
    boost::shared_ptr<OutputPortInterface<T > > Create(string name, RTT::TaskContext *tc, const std::string& prefix)
    {
        OutputPortInterface<T > * instance = NULL;

        // find name in the registry and call factory method.
        typename std::map<string, OutputPortInterface<T >* (*)(RTT::TaskContext *tc, const std::string& prefix) >::const_iterator it = factoryFunctionRegistry.find(name);
        if(it != factoryFunctionRegistry.end())
            instance = it->second(tc, prefix);

        // wrap instance in a shared ptr and return
        if(instance != NULL)
            return boost::shared_ptr<OutputPortInterface<T > >(instance);
        else
            return boost::shared_ptr<OutputPortInterface<T > >();
    }

    void RegisterFactoryFunction(string name, OutputPortInterface<T >* (*classFactoryFunction)(RTT::TaskContext *tc, const std::string& prefix) )
    {
        // register the class factory function
        factoryFunctionRegistry[name] = classFactoryFunction;
    }

    static OutputPortInterfaceFactory* Instance()
    {
        static OutputPortInterfaceFactory factory;
        return &factory;
    }
};

template<class T>
OutputPortInterface<typename T::Container_ >* OutputPortInterfaceFactoryFunction(RTT::TaskContext *tc, const std::string& prefix) {
    return new T(tc, prefix);
}

template<class T>
class OutputPortInterfaceRegistrar {
public:
    OutputPortInterfaceRegistrar(const std::string& name)
    {
        // register the class factory function 
        OutputPortInterfaceFactory<typename T::Container_ >::Instance()->RegisterFactoryFunction(name, OutputPortInterfaceFactoryFunction<T >);
    }
};

};  // namespace interface_ports

#define LITERAL_input_registrar_(X) input_registrar_##X
#define EXPAND_input_registrar_(X) LITERAL_input_registrar_(X)

#define LITERAL_output_registrar_(X) output_registrar_##X
#define EXPAND_output_registrar_(X) LITERAL_output_registrar_(X)

#define REGISTER_InputPortInterface( CLASS, NAME ) static interface_ports::InputPortInterfaceRegistrar<CLASS > EXPAND_input_registrar_(__LINE__)(NAME);

#define REGISTER_OutputPortInterface( CLASS, NAME ) static interface_ports::OutputPortInterfaceRegistrar<CLASS > EXPAND_output_registrar_(__LINE__)(NAME);

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


#endif  // __COMMON_INTERFACES_INTERFACE_PORTS_H__

