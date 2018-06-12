/*
    Made with ❤ by srinSkit.
    Created on 31 May 2018.
*/

#include <dlfcn.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "ros/ros.h"
#include "Crypt.h"
#include "std_msgs/String.h"

#define AS_PORT 8000
bool called_init = false;
std::string source_name, node_name;
Crypt myCrypt;
SecureSock::Client client(&myCrypt);

void term_mudboxer();

void init_mudboxer() {
    if (called_init) return;
    called_init = true;
    printf("%s\n", "RUNNING INSIDE WRAPPER");
    myCrypt.initialize("ROSS");
    source_name = getenv("ROSS_SOURCE_NAME");
    node_name = getenv("ROSS_NODE_NAME");
    myCrypt.add_cert("root", "/home/srinag/Projects/srinskitCA/srinskitCA.pem");
    myCrypt.load_private_key(getenv("ROSS_NODE_KEY"), getenv("ROSS_NODE_PASS"));
    myCrypt.load_my_cert(getenv("ROSS_NODE_CERT"), true);
    client.init();
    if (!client.connect("AuthServer", "localhost", AS_PORT)) {
        printf("Couldn't connect to AuthServer\n");
        term_mudboxer();
        exit(EXIT_FAILURE);
    }
}

void term_mudboxer() {
    // Todo: Use a different param to avoid calling terminate multiple times
    if (!called_init)return;
    called_init = false;
    client.close();
    client.terminate();
    myCrypt.terminate();
    printf("%s\n", "\nBYE!");
}

void mod_pub_ops(ros::AdvertiseOptions &ops) {
    std::stringstream msg;
    msg << "REG_PUB;" << ops.topic.c_str() << ";" << ops.datatype.c_str();
    client.write(msg.str());
    std::string key;
    client.read(key);
    myCrypt.save_aes_key(ops.topic, key);
    ops.datatype = ros::message_traits::DataType<std_msgs::String>::value();
    ops.md5sum = ros::message_traits::MD5Sum<std_msgs::String>::value();
    ops.message_definition = ros::message_traits::definition<std_msgs::String>();
    ops.has_header = ros::message_traits::hasHeader<std_msgs::String>();
}


namespace ros {
    void init(int &argc, char **argv, const std::string &name, uint32_t options) {
        init_mudboxer();
        typedef void (*type)(int &argc, char **argv, const std::string &name, uint32_t options);
        auto ptr = (type) dlsym(RTLD_NEXT, "_ZN3ros4initERiPPcRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEj");
        ptr(argc, argv, name, options);
    }

    void shutdown() {
        term_mudboxer();
        typedef void (*type)();
        auto ptr = (type) dlsym(RTLD_NEXT, "_ZN3ros8shutdownEv");
        ptr();
    }

    void Publisher::publish(const boost::function<SerializedMessage(void)> &serfunc, SerializedMessage &m) const {
//        m.type_info = &typeid(std_msgs::String);
        printf("%zu %s\n", m.num_bytes, (char *)m.buf.get());
        
        typedef void (*type)(const Publisher *, const boost::function<SerializedMessage(void)> &,
                             ros::SerializedMessage &);
        auto ptr = (type) dlsym(RTLD_NEXT,
                                "_ZNK3ros9Publisher7publishERKN5boost8functionIFNS_17SerializedMessageEvEEERS3_");
        return ptr(this, serfunc, m);
    }

    Publisher NodeHandle::advertise(AdvertiseOptions &ops) {
        mod_pub_ops(ops);
        typedef Publisher (*type)(NodeHandle *, AdvertiseOptions &);
        auto ptr = (type) dlsym(RTLD_NEXT, "_ZN3ros10NodeHandle9advertiseERNS_16AdvertiseOptionsE");
        auto pub_er = ptr(this, ops);
        // Bypass ROS_ASSERT_MSG at publisher.h line 79
        pub_er.impl_->md5sum_ = "*";
        return pub_er;
    }

    Subscriber NodeHandle::subscribe(SubscribeOptions &ops) {
//        mySub(ops);
        typedef Subscriber (*type)(NodeHandle *, SubscribeOptions &);
        auto ptr = (type) dlsym(RTLD_NEXT, "_ZN3ros10NodeHandle9subscribeERNS_16SubscribeOptionsE");
        return ptr(this, ops);
    }

} // namespace ros