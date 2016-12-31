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

#include <future>
#include <limits.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include "common_interfaces_test_msgs/Container.h"
#include <rtt/extras/SlaveActivity.hpp>

#include "gtest/gtest.h"

#include "test_deployer.h"
#include "container_utils.h"

namespace message_concate_tests {

using namespace common_interfaces_test_msgs;

// Tests MessageRx/MessageTx classes
TEST(MessageRxTxTest, CreateSync) {
    const std::string fifo_rx_status("/tmp/common_interfaces_test_fifo_rx_status");
    mkfifo(fifo_rx_status.c_str(), 0666);

    int rx_status_pipe_fd;
    rx_status_pipe_fd = open(fifo_rx_status.c_str() , O_RDONLY | O_NONBLOCK);
    if (rx_status_pipe_fd < 0) {
        std::cout << "could not open pipe \'" << fifo_rx_status << "\'" << std::endl;
    }
    EXPECT_TRUE(rx_status_pipe_fd > 0);

    auto fut_rx = std::async(std::launch::async,
        [fifo_rx_status] { return system((std::string("rosrun common_interfaces_test deployer_rx stop 1 0 ") + fifo_rx_status).c_str()); });
    auto fut_tx = std::async(std::launch::async,
        [] { return system("rosrun common_interfaces_test deployer_tx stop 1 0 "); });

    struct timespec time_start, time_curr;
    clock_gettime(CLOCK_REALTIME, &time_start);
    DeployerRxStatus status = DeployerRxStatus();
    while (true) {
//        DeployerRxStatus status_prev = status;
        int nbytes = read(rx_status_pipe_fd, &status , sizeof(status));
//        if (nbytes > 0) {
//            if (!status.isEqual(status_prev)) {
//                std::cout << (status.any_read_successful_?"true":"false") << " "
//                    << (status.checksum_ok_?"true":"false") << " "
//                    << (status.all_reads_successful_?"true":"false") << " " << std::endl;
//            }
//        }
        if (fut_rx.wait_for(std::chrono::seconds(0)) == std::future_status::ready && fut_tx.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            break;
        }
    }

//    std::cout << fut_rx.get() << " " << fut_tx.get() << std::endl;
//    EXPECT_TRUE(fut_rx.get() == 0);
//    EXPECT_TRUE(fut_tx.get() == 0);

    EXPECT_TRUE(status.any_read_successful_);
    EXPECT_TRUE(status.checksum_ok_);
//    EXPECT_TRUE(status.all_reads_successful_);
//            std::cout << (status.any_read_successful_?"true":"false") << " "
//                << (status.checksum_ok_?"true":"false") << " "
//                << (status.all_reads_successful_?"true":"false") << " " << std::endl;
    close(rx_status_pipe_fd);
}

/*
template <typename T>
class TestComponentOut : public RTT::TaskContext {

public:

    explicit TestComponentOut(const std::string& name)
        : TaskContext(name, PreOperational)
        , port("msg_OUTPORT", false)
    {
        this->ports()->addPort(port);
    }

    bool configureHook() {
        return true;
    }

    bool startHook() {
        return true;
    }

    void updateHook() {
        port.write(msg_);
    }

    bool connectPort(RTT::TaskContext* other) {
        RTT::base::PortInterface* pi = other->ports()->getPort("msg_INPORT");
        if (!pi || !port.connectTo( pi )) {
            return false;
        }
        return true;
    }

    void setData(const T& msg) {
        msg_ = msg;
    }

private:
    T msg_;
    RTT::OutputPort<T > port;
};

template <typename T>
class TestComponentIn : public RTT::TaskContext {

public:

    explicit TestComponentIn(const std::string& name)
        : TaskContext(name, PreOperational)
        , port("msg_INPORT")
        , read_successful_(false)
    {
        this->ports()->addPort(port);
    }

    bool configureHook() {
        return true;
    }

    bool startHook() {
        return true;
    }

    void stopHook() {
        std::cout << "TestComponentIn.stopHook: state: " << (int)getTaskState() << std::endl;
        if (getTaskState() == TaskContext::Exception || getTaskState() == TaskContext::RunTimeError) {
            recover();
            std::cout << "TestComponentIn.stopHook: state after recover: " << (int)getTaskState() << std::endl;
        }
    }

    void updateHook() {
        read_successful_ = (port.read(msg_) == RTT::NewData);
    }

    bool connectPort(RTT::TaskContext* other) {
        if (!port.connectTo( other->ports()->getPort("msg_OUTPORT") )) {
            return false;
        }
        return true;
    }

    Container getData() {
        return msg_;
    }

    bool isReadSuccessful() const {
        return read_successful_;
    }

private:
    T msg_;

    bool read_successful_;

    RTT::InputPort<T > port;
};

uint32_t randMinMax(uint32_t min, uint32_t max) {
    return rand()%(max - min + 1) + min;
}

uint32_t calculateChecksum(Container& cont_in) {
    return
        cont_in.cont1.sub1.field1 +
        cont_in.cont1.sub1.field2 +
        cont_in.cont1.sub2.field1 +
        cont_in.cont1.sub2.field2 +
        cont_in.cont1.field1 +
        cont_in.cont1.field2 +
        cont_in.cont2.sub1.field1 +
        cont_in.cont2.sub1.field2 +
        cont_in.cont2.sub2.field1 +
        cont_in.cont2.sub2.field2 +
        cont_in.cont2.field1 +
        cont_in.cont2.field2 +
        cont_in.cont3.sub1.field1 +
        cont_in.cont3.sub1.field2 +
        cont_in.cont3.sub2.field1 +
        cont_in.cont3.sub2.field2 +
        cont_in.cont3.field1 +
        cont_in.cont3.field2 +
        cont_in.cont4.sub1.field1 +
        cont_in.cont4.sub1.field2 +
        cont_in.cont4.sub2.field1 +
        cont_in.cont4.sub2.field2 +
        cont_in.cont4.field1 +
        cont_in.cont4.field2 +
        cont_in.field1;
}

void randomContainerData(Container& cont_in) {
  cont_in.cont1.sub1.field1 = randMinMax(1, 100);
  cont_in.cont1.sub1.field1_valid = true;
  cont_in.cont1.sub1.field2 = randMinMax(1, 100);
  cont_in.cont1.sub1_valid = true;
  cont_in.cont1.sub2.field1 = randMinMax(1, 100);
  cont_in.cont1.sub2.field1_valid = true;
  cont_in.cont1.sub2.field2 = randMinMax(1, 100);
  cont_in.cont1.field1 = randMinMax(1, 100);
  cont_in.cont1.field1_valid = true;
  cont_in.cont1.field2 = randMinMax(1, 100);
  cont_in.cont1_valid = true;
  cont_in.cont2.sub1.field1 = randMinMax(1, 100);
  cont_in.cont2.sub1.field2 = randMinMax(1, 100);
  cont_in.cont2.sub2.field1 = randMinMax(1, 100);
  cont_in.cont2.sub2.field2 = randMinMax(1, 100);
  cont_in.cont2.field1 = randMinMax(1, 100);
  cont_in.cont2.field1_valid = true;
  cont_in.cont2.field2 = randMinMax(1, 100);
  cont_in.cont3.sub1.field1 = randMinMax(1, 100);
  cont_in.cont3.sub1.field2 = randMinMax(1, 100);
  cont_in.cont3.sub2.field1 = randMinMax(1, 100);
  cont_in.cont3.sub2.field2 = randMinMax(1, 100);
  cont_in.cont3.field1 = randMinMax(1, 100);
  cont_in.cont3.field2 = randMinMax(1, 100);
  cont_in.cont3_valid = true;
  cont_in.cont4.sub1.field1 = randMinMax(1, 100);
  cont_in.cont4.sub1.field2 = randMinMax(1, 100);
  cont_in.cont4.sub1_valid = true;
  cont_in.cont4.sub2.field1 = randMinMax(1, 100);
  cont_in.cont4.sub2.field2 = randMinMax(1, 100);
  cont_in.cont4.field1 = randMinMax(1, 100);
  cont_in.cont4.field2 = randMinMax(1, 100);
  cont_in.field1 = randMinMax(1, 100);
  cont_in.field1_valid = true;

  // checksum
  cont_in.field2 = calculateChecksum(cont_in);
}

class TestThread {
protected:
    pthread_mutex_t mutex_;
    bool initialized_;
    bool loop_started_;
    bool loop_stopped_;
    pthread_t thr_;

    typedef void*(*ThreadFuncPtr)(void*);
    ThreadFuncPtr thread_func_;
    bool active_;

public:

    explicit TestThread(ThreadFuncPtr thread_func)
        : mutex_(PTHREAD_MUTEX_INITIALIZER)
        , initialized_(false)
        , loop_started_(false)
        , loop_stopped_(false)
        , thread_func_(thread_func)
        , active_(false) {
    }

    void start() {
        active_ = true;
        pthread_create(&thr_, NULL, thread_func_, this);
    }

    void join() {
        pthread_join(thr_, NULL);
    }

    void cancel() {
        active_ = false;
        pthread_cancel(thr_);
    }

    void setInitialized(bool initialized) {
        pthread_mutex_lock(&mutex_);
        initialized_ = initialized;
        pthread_mutex_unlock(&mutex_);
    }

    bool isInitialized() {
        pthread_mutex_lock(&mutex_);
        bool initialized = initialized_;
        pthread_mutex_unlock(&mutex_);
        return initialized;
    }

    bool isActive() const {
        return active_;
    }

    void startLoop() {
        pthread_mutex_lock(&mutex_);
        loop_started_ = true;
        pthread_mutex_unlock(&mutex_);
    }

    bool isLoopStarted() {
        pthread_mutex_lock(&mutex_);
        bool loop_started = loop_started_;
        pthread_mutex_unlock(&mutex_);
        return loop_started;
    }

    void stopLoop() {
        pthread_mutex_lock(&mutex_);
        loop_stopped_ = true;
        pthread_mutex_unlock(&mutex_);
    }

    bool isLoopStopped() {
        pthread_mutex_lock(&mutex_);
        bool loop_stopped = loop_stopped_;
        pthread_mutex_unlock(&mutex_);
        return loop_stopped;
    }

};

class TxThread : public TestThread {
public:

    TxThread(boost::shared_ptr<message_tests::TestDeployer > d)
        : TestThread(thread_func)
        , kick_out_ok_(false)
        , d_(d) {
    }

    void cleanup() {
//        message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();
        kick_out_ok_ = d_->getDc()->kickOutComponent("tx");
    }

    bool kick_out_ok_;

protected:
    boost::shared_ptr<message_tests::TestDeployer > d_;

    static void* thread_func(void* arg) {

        TxThread* this_ = static_cast<TxThread* >(arg);

        bool init_ok = this_->d_->getDc()->loadComponent("tx", "common_interfaces_test_msgs_ContainerTx");
        RTT::TaskContext *tx = this_->d_->getDc()->getPeer("tx");
        init_ok &= (tx != NULL);
        tx->setActivity( new RTT::extras::SlaveActivity() );
        RTT::Property<std::string >* tx_channel_name = dynamic_cast<RTT::Property<std::string >* >(tx->getProperty("channel_name"));
        init_ok &= (tx_channel_name != NULL );
        tx_channel_name->set("channel6");
        init_ok &= tx->configure();

        TestComponentOut<common_interfaces_test_msgs::Container > test_out("test_out");
        test_out.setActivity( new RTT::extras::SlaveActivity() );
        init_ok &= test_out.connectPort(tx);
        init_ok &= test_out.configure();

        init_ok &= tx->start();
        init_ok &= test_out.start();

        this_->setInitialized(init_ok);

        if (!init_ok) {
            return NULL;
        }

        while (this_->isLoopStarted() == false) {
            usleep(1000);
        }

        while (this_->isLoopStopped() == false) {
//            usleep(1000);
            Container cont;
            randomContainerData(cont);
            test_out.setData(cont);
            test_out.getActivity()->execute();
            tx->getActivity()->execute();
        }

        // put zero data in buffer
//        for (int i = 0; i < 100; ++i) {
        {
            Container cont = Container();
            test_out.setData(cont);
            test_out.getActivity()->execute();
            tx->getActivity()->execute();
        }
//        }

        // stop & cleanup
        test_out.stop();
        test_out.cleanup();

        tx->stop();
        tx->cleanup();

        this_->kick_out_ok_ = this_->d_->getDc()->kickOutComponent("tx");

        this_->active_ = false;
        return NULL;
    }
};

class RxThread : public TestThread {
public:

    RxThread(boost::shared_ptr<message_tests::TestDeployer > d)
        : TestThread(thread_func)
        , all_reads_successful_(false)
        , any_read_successful_(false)
        , checksum_ok_(false)
        , kick_out_ok_(false)
        , d_(d) {
    }

    bool allReadsSuccessful() const {
        return all_reads_successful_;
    }

    bool anyReadSuccessful() const {
        return any_read_successful_;
    }

    bool checksumOk() const {
        return checksum_ok_;
    }

    void cleanup() {
//        message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();
        kick_out_ok_ = d_->getDc()->kickOutComponent("rx");
    }

    bool kick_out_ok_;

protected:
    bool all_reads_successful_;
    bool any_read_successful_;
    bool checksum_ok_;
    boost::shared_ptr<message_tests::TestDeployer > d_;

    static void* thread_func(void* arg) {

        RxThread* this_ = static_cast<RxThread* >(arg);

        bool init_ok = this_->d_->getDc()->loadComponent("rx", "common_interfaces_test_msgs_ContainerRx");
        RTT::TaskContext *rx = this_->d_->getDc()->getPeer("rx");
        init_ok &= (rx != NULL);
        rx->setActivity( new RTT::extras::SlaveActivity() );
        RTT::Property<std::string >* rx_channel_name = dynamic_cast<RTT::Property<std::string >* >(rx->getProperty("channel_name"));
        init_ok &= ( rx_channel_name != NULL );
        rx_channel_name->set("channel6");
//        std::cout << "RxThread: a" <<std::endl;
        init_ok &= ( rx->configure() );
//        std::cout << "RxThread: b" <<std::endl;

        TestComponentIn<common_interfaces_test_msgs::Container > test_in("test_in");
        test_in.setActivity( new RTT::extras::SlaveActivity() );
        init_ok &= ( test_in.connectPort(rx) );
        init_ok &= ( test_in.configure() );

        init_ok &= ( rx->start() );
        init_ok &= ( test_in.start() );

        this_->setInitialized(init_ok);

        if (!init_ok) {
            return NULL;
        }

        while (this_->isLoopStarted() == false) {
            usleep(1000);
        }

        this_->all_reads_successful_ = true;
        this_->any_read_successful_ = false;
        this_->checksum_ok_ = true;
        while (this_->isLoopStopped() == false) {
//            usleep(1000);
            if (rx->getTaskState() != RTT::TaskContext::Running) {
                break;
            }

            rx->getActivity()->execute();
            std::cout << "rx.state: " << (int)rx->getTaskState() << std::endl;
            std::cout << "d.state: " << (int)this_->d_->getDc()->getTaskState() << std::endl;


            test_in.getActivity()->execute();
            if (test_in.isReadSuccessful()) {
                this_->any_read_successful_ = true;
                Container cont = test_in.getData();
                if (cont.field2 != calculateChecksum(cont)) {
                    this_->checksum_ok_ = false;
                }
            }
            else {
                this_->all_reads_successful_ = false;
            }
        }


        // stop & cleanup
        test_in.stop();
        test_in.cleanup();

        rx->stop();
        rx->cleanup();

        this_->kick_out_ok_ = this_->d_->getDc()->kickOutComponent("rx");

        this_->active_ = false;
    }
};
*/
// Tests for class MessageRx/MessageTx.
/*
// Tests MessageRx/MessageTx classes
TEST(MessageRxTxTest, CreateSync) {

    message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();

    EXPECT_TRUE( d.getDc()->import("rtt_common_interfaces_test_subsystem_ports") );

    for (int i = 0; i < 100; ++i) {
//        std::cout << "CreateSync a: " << i << std::endl;
        TxThread tx;
        tx.start();
        while (!tx.isInitialized() && tx.isActive()) {
            usleep(1000);
        }
        EXPECT_TRUE(tx.isInitialized());
        EXPECT_TRUE(tx.isActive());
//        std::cout << "CreateSync b: " << i << std::endl;

        RxThread rx;
        rx.start();
        while (!rx.isInitialized() && rx.isActive()) {
            usleep(1000);
        }
        EXPECT_TRUE(rx.isInitialized());
        EXPECT_TRUE(rx.isActive());

//        std::cout << "CreateSync c: " << i << std::endl;

        tx.startLoop();
        rx.startLoop();

//        std::cout << "CreateSync d: " << i << std::endl;

        usleep(1000+(rand()%1000));

//        tx.cancel();
//        rx.cancel();

        rx.stopLoop();
        rx.join();

//        std::cout << "CreateSync e: " << i << std::endl;

        tx.stopLoop();
        tx.join();

//        std::cout << "CreateSync f: " << i << std::endl;

        EXPECT_TRUE(rx.anyReadSuccessful());
        EXPECT_TRUE(rx.allReadsSuccessful());
        EXPECT_TRUE(rx.checksumOk());

        d.getDc()->kickOutAll();
    }
}
*/
/*
// Tests MessageRx/MessageTx classes
TEST(MessageRxTxTest, CancelRx) {
    boost::shared_ptr<message_tests::TestDeployer > d(new message_tests::TestDeployer("test_depl"));
    EXPECT_TRUE( d->getDc()->import("rtt_common_interfaces_test_subsystem_ports") );

    TxThread tx(d);
    tx.start();
    while (!tx.isInitialized() && tx.isActive()) {
        usleep(1000);
    }
    EXPECT_TRUE(tx.isInitialized());
    EXPECT_TRUE(tx.isActive());

    tx.startLoop();

    for (int i = 0; i < 100; ++i) {
        RxThread rx(d);
        rx.start();
        while ( !rx.isInitialized() && rx.isActive() ) {
            usleep(1000);
        }

        EXPECT_TRUE(rx.isInitialized());
        EXPECT_TRUE(rx.isActive());

        rx.startLoop();

        usleep(2000+(rand()%1000));

        rx.cancel();
        rx.join();
        rx.cleanup();

        EXPECT_TRUE(rx.anyReadSuccessful());
        EXPECT_TRUE(rx.checksumOk());
    }

//    tx.stopLoop();

//    tx.join();
}
*/
/*
TEST(MessageRxTxTest, CancelTx) {
    boost::shared_ptr<message_tests::TestDeployer > d(new message_tests::TestDeployer("test_depl"));
    EXPECT_TRUE( d->getDc()->import("rtt_common_interfaces_test_subsystem_ports") );

    RxThread rx(d);
    rx.start();
    while ( !rx.isInitialized() && rx.isActive() ) {
        usleep(1000);
    }

    EXPECT_TRUE(rx.isInitialized());
    EXPECT_TRUE(rx.isActive());

    rx.startLoop();

    for (int i = 0; i < 100; ++i) {
        TxThread tx(d);
        tx.start();
        while (!tx.isInitialized() && tx.isActive()) {
            usleep(1000);
        }
        EXPECT_TRUE(tx.isInitialized());
        EXPECT_TRUE(tx.isActive());

        tx.startLoop();

        usleep(2000+(rand()%1000));

        tx.cancel();
        tx.join();
        tx.cleanup();
    }

    rx.stopLoop();

    rx.join();

    EXPECT_TRUE(rx.anyReadSuccessful());
    EXPECT_TRUE(rx.checksumOk());
}
*/

/*
// Tests MessageRx/MessageTx classes
TEST(MessageRxTxTest, CreateTx) {

    message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();

    EXPECT_TRUE( d.getDc()->import("rtt_common_interfaces_test_subsystem_ports") );

    RxThread rx;
    rx.start();
    while (rx.isInitialized() == false) {
        usleep(1000);
    }

    rx.startLoop();

    for (int i = 0; i < 100; ++i) {

        TxThread tx;
        tx.start();
        while (tx.isInitialized() == false) {
            usleep(1000);
        }

        tx.startLoop();

        usleep(1+(rand()%1000));

        tx.stopLoop();
        tx.join();
    }
    rx.stopLoop();
    rx.join();

    EXPECT_TRUE(rx.anyReadSuccessful());
    EXPECT_TRUE(!rx.allReadsSuccessful());
    EXPECT_TRUE(rx.checksumOk());

    d.getDc()->kickOutAll();
}


*/
/*
// Tests MessageConcate class for data valid on some input ports.
TEST(MessageConcateTest, InvalidCaughtOnTheSameLevel) {

  message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();

  EXPECT_TRUE( d.getDc()->import("rtt_common_interfaces_test_subsystem_ports") );
  EXPECT_TRUE(d.getDc()->loadComponent("concate", "common_interfaces_test_msgs_ContainerConcate"));
  RTT::TaskContext *concate = d.getDc()->getPeer("concate");
  EXPECT_TRUE(concate != NULL);

  concate->setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( concate->configure() );
  EXPECT_TRUE( concate->start() );
  EXPECT_EQ( concate->ports()->getPortNames().size(), 24);

  // component that writes data on input ports of the component under test
  TestComponentIn test_in("test_in");
  test_in.setActivity( new RTT::extras::SlaveActivity() );
  std::vector<std::string > not_connected_ports;
  not_connected_ports.push_back("cont1_sub1_field1");
  EXPECT_TRUE( test_in.connectPorts(concate, not_connected_ports) );
  EXPECT_TRUE( test_in.configure() );
  EXPECT_TRUE( test_in.start() );

  // component that reads data from output port of the component under test
  TestComponentOut test_out("test_out");
  test_out.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_out.connectPorts(concate) );
  EXPECT_TRUE( test_out.configure() );
  EXPECT_TRUE( test_out.start() );

  // generate some data
  Container cont_in;
  initContainerData(cont_in);

  // load the data into test_in component
  test_in.setData( cont_in );

  // execute all components
  test_in.getActivity()->execute();
  concate->getActivity()->execute();
  test_out.getActivity()->execute();

  // get the result data
  Container cont_out = test_out.getData();

  // check the data

  EXPECT_EQ(cont_out.cont1.sub1.field1,         0);         // invalid values are set to default values
  EXPECT_EQ(cont_out.cont1.sub1.field1_valid,   false);     // this was not connected, so it is invalid
  EXPECT_EQ(cont_out.cont1.sub1.field2,         cont_in.cont1.sub1.field2);
  EXPECT_EQ(cont_out.cont1.sub1_valid,          true);
  EXPECT_EQ(cont_out.cont1.sub2.field1,         cont_in.cont1.sub2.field1);
  EXPECT_EQ(cont_out.cont1.sub2.field1_valid,   true);
  EXPECT_EQ(cont_out.cont1.sub2.field2,         cont_in.cont1.sub2.field2);
  EXPECT_EQ(cont_out.cont1.field1,              cont_in.cont1.field1);
  EXPECT_EQ(cont_out.cont1.field1_valid,        true);
  EXPECT_EQ(cont_out.cont1.field2,              cont_in.cont1.field2);
  EXPECT_EQ(cont_out.cont1_valid,               true);

  EXPECT_EQ(cont_out.cont2.sub1.field1,         cont_in.cont2.sub1.field1);
  EXPECT_EQ(cont_out.cont2.sub1.field1_valid,   true);
  EXPECT_EQ(cont_out.cont2.sub1.field2,         cont_in.cont2.sub1.field2);
  EXPECT_EQ(cont_out.cont2.sub1_valid,          true);
  EXPECT_EQ(cont_out.cont2.sub2.field1,         cont_in.cont2.sub2.field1);
  EXPECT_EQ(cont_out.cont2.sub2.field1_valid,   true);
  EXPECT_EQ(cont_out.cont2.sub2.field2,         cont_in.cont2.sub2.field2);
  EXPECT_EQ(cont_out.cont2.field1,              cont_in.cont2.field1);
  EXPECT_EQ(cont_out.cont2.field1_valid,        true);
  EXPECT_EQ(cont_out.cont2.field2,              cont_in.cont2.field2);

  EXPECT_EQ(cont_out.cont3.sub1.field1,         cont_in.cont3.sub1.field1);
  EXPECT_EQ(cont_out.cont3.sub1.field1_valid,   false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont3.sub1.field2,         cont_in.cont3.sub1.field2);
  EXPECT_EQ(cont_out.cont3.sub1_valid,          false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont3.sub2.field1,         cont_in.cont3.sub2.field1);
  EXPECT_EQ(cont_out.cont3.sub2.field1_valid,   false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont3.sub2.field2,         cont_in.cont3.sub2.field2);
  EXPECT_EQ(cont_out.cont3.field1,              cont_in.cont3.field1);
  EXPECT_EQ(cont_out.cont3.field1_valid,        false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont3.field2,              cont_in.cont3.field2);
  EXPECT_EQ(cont_out.cont3_valid,               true);

  EXPECT_EQ(cont_out.cont4.sub1.field1,         cont_in.cont4.sub1.field1);
  EXPECT_EQ(cont_out.cont4.sub1.field1_valid,   false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont4.sub1.field2,         cont_in.cont4.sub1.field2);
  EXPECT_EQ(cont_out.cont4.sub1_valid,          true);                      // this field is set manually
  EXPECT_EQ(cont_out.cont4.sub2.field1,         cont_in.cont4.sub2.field1);
  EXPECT_EQ(cont_out.cont4.sub2.field1_valid,   false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont4.sub2.field2,         cont_in.cont4.sub2.field2);
  EXPECT_EQ(cont_out.cont4.field1,              cont_in.cont4.field1);
  EXPECT_EQ(cont_out.cont4.field1_valid,        false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont4.field2,              cont_in.cont4.field2);

  EXPECT_EQ(cont_out.field1,                    cont_in.field1);
  EXPECT_EQ(cont_out.field1_valid,              true);

  EXPECT_EQ(cont_out.field2,                    cont_in.field2);

  // stop & cleanup
  test_in.stop();
  test_in.cleanup();

  test_out.stop();
  test_out.cleanup();

  concate->stop();
  concate->cleanup();

  d.getDc()->kickOutAll();
}

// Tests MessageConcate class for data valid on some input ports.
TEST(MessageConcateTest, InvalidCaughtOnHigherLevel) {

  message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();

  EXPECT_TRUE( d.getDc()->import("rtt_common_interfaces_test_subsystem_ports") );
  EXPECT_TRUE(d.getDc()->loadComponent("concate", "common_interfaces_test_msgs_ContainerConcate"));
  RTT::TaskContext *concate = d.getDc()->getPeer("concate");
  EXPECT_TRUE(concate != NULL);

  concate->setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( concate->configure() );
  EXPECT_TRUE( concate->start() );
  EXPECT_EQ( concate->ports()->getPortNames().size(), 24);

  // component that writes data on input ports of the component under test
  TestComponentIn test_in("test_in");
  test_in.setActivity( new RTT::extras::SlaveActivity() );
  std::vector<std::string > not_connected_ports;
  not_connected_ports.push_back("cont1_sub1_field2");
  EXPECT_TRUE( test_in.connectPorts(concate, not_connected_ports) );
  EXPECT_TRUE( test_in.configure() );
  EXPECT_TRUE( test_in.start() );

  // component that reads data from output port of the component under test
  TestComponentOut test_out("test_out");
  test_out.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_out.connectPorts(concate) );
  EXPECT_TRUE( test_out.configure() );
  EXPECT_TRUE( test_out.start() );

  // generate some data
  Container cont_in;
  initContainerData(cont_in);

  // load the data into test_in component
  test_in.setData( cont_in );

  // execute all components
  test_in.getActivity()->execute();
  concate->getActivity()->execute();
  test_out.getActivity()->execute();

  // get the result data
  Container cont_out = test_out.getData();

  // check the data

  EXPECT_EQ(cont_out.cont1.sub1.field1,         0);     // invalid values are set to default values
  EXPECT_EQ(cont_out.cont1.sub1.field1_valid,   false); // invalid values are set to default values
  EXPECT_EQ(cont_out.cont1.sub1.field2,         0);     // this was not connected, so the whole container is invalid
  EXPECT_EQ(cont_out.cont1.sub1_valid,          false);
  EXPECT_EQ(cont_out.cont1.sub2.field1,         cont_in.cont1.sub2.field1);
  EXPECT_EQ(cont_out.cont1.sub2.field1_valid,   true);
  EXPECT_EQ(cont_out.cont1.sub2.field2,         cont_in.cont1.sub2.field2);
  EXPECT_EQ(cont_out.cont1.field1,              cont_in.cont1.field1);
  EXPECT_EQ(cont_out.cont1.field1_valid,        true);
  EXPECT_EQ(cont_out.cont1.field2,              cont_in.cont1.field2);
  EXPECT_EQ(cont_out.cont1_valid,               true);

  EXPECT_EQ(cont_out.cont2.sub1.field1,         cont_in.cont2.sub1.field1);
  EXPECT_EQ(cont_out.cont2.sub1.field1_valid,   true);
  EXPECT_EQ(cont_out.cont2.sub1.field2,         cont_in.cont2.sub1.field2);
  EXPECT_EQ(cont_out.cont2.sub1_valid,          true);
  EXPECT_EQ(cont_out.cont2.sub2.field1,         cont_in.cont2.sub2.field1);
  EXPECT_EQ(cont_out.cont2.sub2.field1_valid,   true);
  EXPECT_EQ(cont_out.cont2.sub2.field2,         cont_in.cont2.sub2.field2);
  EXPECT_EQ(cont_out.cont2.field1,              cont_in.cont2.field1);
  EXPECT_EQ(cont_out.cont2.field1_valid,        true);
  EXPECT_EQ(cont_out.cont2.field2,              cont_in.cont2.field2);

  EXPECT_EQ(cont_out.cont3.sub1.field1,         cont_in.cont3.sub1.field1);
  EXPECT_EQ(cont_out.cont3.sub1.field1_valid,   false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont3.sub1.field2,         cont_in.cont3.sub1.field2);
  EXPECT_EQ(cont_out.cont3.sub1_valid,          false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont3.sub2.field1,         cont_in.cont3.sub2.field1);
  EXPECT_EQ(cont_out.cont3.sub2.field1_valid,   false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont3.sub2.field2,         cont_in.cont3.sub2.field2);
  EXPECT_EQ(cont_out.cont3.field1,              cont_in.cont3.field1);
  EXPECT_EQ(cont_out.cont3.field1_valid,        false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont3.field2,              cont_in.cont3.field2);
  EXPECT_EQ(cont_out.cont3_valid,               true);

  EXPECT_EQ(cont_out.cont4.sub1.field1,         cont_in.cont4.sub1.field1);
  EXPECT_EQ(cont_out.cont4.sub1.field1_valid,   false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont4.sub1.field2,         cont_in.cont4.sub1.field2);
  EXPECT_EQ(cont_out.cont4.sub1_valid,          true);                      // this field is set manually
  EXPECT_EQ(cont_out.cont4.sub2.field1,         cont_in.cont4.sub2.field1);
  EXPECT_EQ(cont_out.cont4.sub2.field1_valid,   false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont4.sub2.field2,         cont_in.cont4.sub2.field2);
  EXPECT_EQ(cont_out.cont4.field1,              cont_in.cont4.field1);
  EXPECT_EQ(cont_out.cont4.field1_valid,        false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont4.field2,              cont_in.cont4.field2);

  EXPECT_EQ(cont_out.field1,                    cont_in.field1);
  EXPECT_EQ(cont_out.field1_valid,              true);

  EXPECT_EQ(cont_out.field2,                    cont_in.field2);

  // stop & cleanup
  test_in.stop();
  test_in.cleanup();

  test_out.stop();
  test_out.cleanup();

  concate->stop();
  concate->cleanup();

  d.getDc()->kickOutAll();
}


// Tests MessageConcate class for data valid on some input ports.
TEST(MessageConcateTest, InvalidCaughtOnHighestLevel) {

  message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();

  EXPECT_TRUE( d.getDc()->import("rtt_common_interfaces_test_subsystem_ports") );
  EXPECT_TRUE(d.getDc()->loadComponent("concate", "common_interfaces_test_msgs_ContainerConcate"));
  RTT::TaskContext *concate = d.getDc()->getPeer("concate");
  EXPECT_TRUE(concate != NULL);

  concate->setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( concate->configure() );
  EXPECT_TRUE( concate->start() );
  EXPECT_EQ( concate->ports()->getPortNames().size(), 24);

  // component that writes data on input ports of the component under test
  TestComponentIn test_in("test_in");
  test_in.setActivity( new RTT::extras::SlaveActivity() );
  std::vector<std::string > not_connected_ports;
  not_connected_ports.push_back("cont2_sub2_field2");
  EXPECT_TRUE( test_in.connectPorts(concate, not_connected_ports) );
  EXPECT_TRUE( test_in.configure() );
  EXPECT_TRUE( test_in.start() );

  // component that reads data from output port of the component under test
  TestComponentOut test_out("test_out");
  test_out.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_out.connectPorts(concate) );
  EXPECT_TRUE( test_out.configure() );
  EXPECT_TRUE( test_out.start() );

  // generate some data
  Container cont_in;
  initContainerData(cont_in);

  // load the data into test_in component
  test_in.setData( cont_in );

  // execute all components
  test_in.getActivity()->execute();
  concate->getActivity()->execute();
  test_out.getActivity()->execute();

  // get the result data
  Container cont_out = test_out.getData();

  // check the data (all values are invalid)
  EXPECT_EQ(cont_out.cont1.sub1.field1,         0);
  EXPECT_EQ(cont_out.cont1.sub1.field1_valid,   false);
  EXPECT_EQ(cont_out.cont1.sub1.field2,         0);
  EXPECT_EQ(cont_out.cont1.sub1_valid,          false);
  EXPECT_EQ(cont_out.cont1.sub2.field1,         0);
  EXPECT_EQ(cont_out.cont1.sub2.field1_valid,   false);
  EXPECT_EQ(cont_out.cont1.sub2.field2,         0);
  EXPECT_EQ(cont_out.cont1.field1,              0.0);
  EXPECT_EQ(cont_out.cont1.field1_valid,        false);
  EXPECT_EQ(cont_out.cont1.field2,              0.0);
  EXPECT_EQ(cont_out.cont1_valid,               false);

  EXPECT_EQ(cont_out.cont2.sub1.field1,         0);
  EXPECT_EQ(cont_out.cont2.sub1.field1_valid,   false);
  EXPECT_EQ(cont_out.cont2.sub1.field2,         0);
  EXPECT_EQ(cont_out.cont2.sub1_valid,          false);
  EXPECT_EQ(cont_out.cont2.sub2.field1,         0);
  EXPECT_EQ(cont_out.cont2.sub2.field1_valid,   false);
  EXPECT_EQ(cont_out.cont2.sub2.field2,         0);
  EXPECT_EQ(cont_out.cont2.field1,              0.0);
  EXPECT_EQ(cont_out.cont2.field1_valid,        false);
  EXPECT_EQ(cont_out.cont2.field2,              0.0);

  EXPECT_EQ(cont_out.cont3.sub1.field1,         0);
  EXPECT_EQ(cont_out.cont3.sub1.field1_valid,   false);
  EXPECT_EQ(cont_out.cont3.sub1.field2,         0);
  EXPECT_EQ(cont_out.cont3.sub1_valid,          false);
  EXPECT_EQ(cont_out.cont3.sub2.field1,         0);
  EXPECT_EQ(cont_out.cont3.sub2.field1_valid,   false);
  EXPECT_EQ(cont_out.cont3.sub2.field2,         0);
  EXPECT_EQ(cont_out.cont3.field1,              0.0);
  EXPECT_EQ(cont_out.cont3.field1_valid,        false);
  EXPECT_EQ(cont_out.cont3.field2,              0.0);

  EXPECT_EQ(cont_out.cont4.sub1.field1,         0);
  EXPECT_EQ(cont_out.cont4.sub1.field1_valid,   false);
  EXPECT_EQ(cont_out.cont4.sub1.field2,         0);
  EXPECT_EQ(cont_out.cont4.sub1_valid,          false);
  EXPECT_EQ(cont_out.cont4.sub2.field1,         0);
  EXPECT_EQ(cont_out.cont4.sub2.field1_valid,   false);
  EXPECT_EQ(cont_out.cont4.sub2.field2,         0);
  EXPECT_EQ(cont_out.cont4.field1,              0.0);
  EXPECT_EQ(cont_out.cont4.field1_valid,        false);
  EXPECT_EQ(cont_out.cont4.field2,              0.0);

  EXPECT_EQ(cont_out.field1,                    0);
  EXPECT_EQ(cont_out.field1_valid,              false);

  EXPECT_EQ(cont_out.field2,                    0);

  // stop & cleanup
  test_in.stop();
  test_in.cleanup();

  test_out.stop();
  test_out.cleanup();

  concate->stop();
  concate->cleanup();

  d.getDc()->kickOutAll();
}


// Tests MessageConcate class for data valid on some input ports.
TEST(MessageConcateTest, InvalidCaughtOnMiddleLevel) {

  message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();

  EXPECT_TRUE( d.getDc()->import("rtt_common_interfaces_test_subsystem_ports") );
  EXPECT_TRUE(d.getDc()->loadComponent("concate", "common_interfaces_test_msgs_ContainerConcate"));
  RTT::TaskContext *concate = d.getDc()->getPeer("concate");
  EXPECT_TRUE(concate != NULL);

  concate->setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( concate->configure() );
  EXPECT_TRUE( concate->start() );
  EXPECT_EQ( concate->ports()->getPortNames().size(), 24);

  // component that writes data on input ports of the component under test
  TestComponentIn test_in("test_in");
  test_in.setActivity( new RTT::extras::SlaveActivity() );
  std::vector<std::string > not_connected_ports;
  not_connected_ports.push_back("cont1_sub2_field2");
  EXPECT_TRUE( test_in.connectPorts(concate, not_connected_ports) );
  EXPECT_TRUE( test_in.configure() );
  EXPECT_TRUE( test_in.start() );

  // component that reads data from output port of the component under test
  TestComponentOut test_out("test_out");
  test_out.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_out.connectPorts(concate) );
  EXPECT_TRUE( test_out.configure() );
  EXPECT_TRUE( test_out.start() );

  // generate some data
  Container cont_in;
  initContainerData(cont_in);

  // load the data into test_in component
  test_in.setData( cont_in );

  // execute all components
  test_in.getActivity()->execute();
  concate->getActivity()->execute();
  test_out.getActivity()->execute();

  // get the result data
  Container cont_out = test_out.getData();

  // check the data (only the high level container is invalid)
  EXPECT_EQ(cont_out.cont1.sub1.field1,         0);
  EXPECT_EQ(cont_out.cont1.sub1.field1_valid,   false);
  EXPECT_EQ(cont_out.cont1.sub1.field2,         0);
  EXPECT_EQ(cont_out.cont1.sub1_valid,          false);
  EXPECT_EQ(cont_out.cont1.sub2.field1,         0);
  EXPECT_EQ(cont_out.cont1.sub2.field1_valid,   false);
  EXPECT_EQ(cont_out.cont1.sub2.field2,         0);
  EXPECT_EQ(cont_out.cont1.field1,              0.0);
  EXPECT_EQ(cont_out.cont1.field1_valid,        false);
  EXPECT_EQ(cont_out.cont1.field2,              0.0);
  EXPECT_EQ(cont_out.cont1_valid,               false);

  EXPECT_EQ(cont_out.cont2.sub1.field1,         cont_in.cont2.sub1.field1);
  EXPECT_EQ(cont_out.cont2.sub1.field1_valid,   true);
  EXPECT_EQ(cont_out.cont2.sub1.field2,         cont_in.cont2.sub1.field2);
  EXPECT_EQ(cont_out.cont2.sub1_valid,          true);
  EXPECT_EQ(cont_out.cont2.sub2.field1,         cont_in.cont2.sub2.field1);
  EXPECT_EQ(cont_out.cont2.sub2.field1_valid,   true);
  EXPECT_EQ(cont_out.cont2.sub2.field2,         cont_in.cont2.sub2.field2);
  EXPECT_EQ(cont_out.cont2.field1,              cont_in.cont2.field1);
  EXPECT_EQ(cont_out.cont2.field1_valid,        true);
  EXPECT_EQ(cont_out.cont2.field2,              cont_in.cont2.field2);

  EXPECT_EQ(cont_out.cont3.sub1.field1,         cont_in.cont3.sub1.field1);
  EXPECT_EQ(cont_out.cont3.sub1.field1_valid,   false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont3.sub1.field2,         cont_in.cont3.sub1.field2);
  EXPECT_EQ(cont_out.cont3.sub1_valid,          false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont3.sub2.field1,         cont_in.cont3.sub2.field1);
  EXPECT_EQ(cont_out.cont3.sub2.field1_valid,   false);
  EXPECT_EQ(cont_out.cont3.sub2.field2,         cont_in.cont3.sub2.field2);
  EXPECT_EQ(cont_out.cont3.field1,              cont_in.cont3.field1);
  EXPECT_EQ(cont_out.cont3.field1_valid,        false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont3.field2,              cont_in.cont3.field2);
  EXPECT_EQ(cont_out.cont3_valid,               true);

  EXPECT_EQ(cont_out.cont4.sub1.field1,         cont_in.cont4.sub1.field1);
  EXPECT_EQ(cont_out.cont4.sub1.field1_valid,   false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont4.sub1.field2,         cont_in.cont4.sub1.field2);
  EXPECT_EQ(cont_out.cont4.sub1_valid,          true);                      // this field is set manually
  EXPECT_EQ(cont_out.cont4.sub2.field1,         cont_in.cont4.sub2.field1);
  EXPECT_EQ(cont_out.cont4.sub2.field1_valid,   false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont4.sub2.field2,         cont_in.cont4.sub2.field2);
  EXPECT_EQ(cont_out.cont4.field1,              cont_in.cont4.field1);
  EXPECT_EQ(cont_out.cont4.field1_valid,        false);                     // this field is set manually
  EXPECT_EQ(cont_out.cont4.field2,              cont_in.cont4.field2);

  EXPECT_EQ(cont_out.field1,                    cont_in.field1);
  EXPECT_EQ(cont_out.field1_valid,              true);

  EXPECT_EQ(cont_out.field2,                    cont_in.field2);

  // stop & cleanup
  test_in.stop();
  test_in.cleanup();

  test_out.stop();
  test_out.cleanup();

  concate->stop();
  concate->cleanup();

  d.getDc()->kickOutAll();
}

// TODO: write unit tests for container ports
*/
};  // namespace message_concate_tests

