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
#include "geometry_msgs/Twist.h"

#define AS_PORT 8000
bool called_init = false;
std::string source_name, node_name;
Crypt myCrypt;
SecureSock::Client client(&myCrypt);
std::map<std::string, std::string> datatype;
std::map<std::string, std::vector<int>> callbacks;

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

typedef std_msgs::String X;
typedef std_msgs::StringConstPtr P;

void mod_msg(const boost::function<ros::SerializedMessage(void)> &serfunc,
             boost::function<ros::SerializedMessage(void)> &n_serfunc, X &ros_str) {
    ros::SerializedMessage m2 = serfunc();
    std::string &msg = ros_str.data;
    msg.append(std::to_string(m2.num_bytes));
    msg.append(1, ';');
    msg.append(reinterpret_cast<char *>(m2.buf.get()), m2.num_bytes);
    n_serfunc = boost::bind(ros::serialization::serializeMessage<X>, boost::ref(ros_str));
    std::cout << msg << std::endl;
}

void unmod_msg(const P &str, ros::SerializedMessage &m) {
    const std::string msg = str->data;
    std::cout << msg.length() << std::endl;
    size_t colon_pos;
//    auto num_bytes = static_cast<size_t>(std::stoi(msg, &colon_pos));
//    m.num_bytes = num_bytes;
//    m.buf = boost::shared_array<uint8_t>(new uint8_t[num_bytes]);
//    msg.copy(reinterpret_cast<char *>(m.buf.get()), num_bytes, colon_pos + 1);
}

void mod_pub_ops(ros::AdvertiseOptions &ops) {
    std::stringstream msg;
    msg << "REG_PUB;" << ops.topic.c_str() << ";" << ops.datatype.c_str();
    client.write(msg.str());
    std::string key;
    client.read(key);
    myCrypt.save_aes_key(ops.topic, key);
    ops.datatype = ros::message_traits::DataType<X>::value();
    ops.md5sum = ros::message_traits::MD5Sum<X>::value();
    ops.message_definition = ros::message_traits::definition<X>();
    ops.has_header = ros::message_traits::hasHeader<X>();
    datatype[ops.topic] = ops.datatype;
}


void trans_callback(const P &str) {
    std::cout << *str << std::endl;
//    ros::SerializedMessage m;
//    unmod_msg(str, m);
}

void mod_sub_ops(ros::SubscribeOptions &ops) {
    ops.template initByFullCallbackType<P>(ops.topic, ops.queue_size, trans_callback);
    datatype[ops.topic] = ops.datatype;
    callbacks[ops.topic].push_back(1);
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
        boost::function<SerializedMessage(void)> n_serfunc;
        X tmp;
        mod_msg(serfunc, n_serfunc, tmp);
        typedef void (*type)(const Publisher *, const boost::function<SerializedMessage(void)> &,
                             ros::SerializedMessage &);
        auto ptr = (type) dlsym(RTLD_NEXT,
                                "_ZNK3ros9Publisher7publishERKN5boost8functionIFNS_17SerializedMessageEvEEERS3_");
        return ptr(this, n_serfunc, m);
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
        mod_sub_ops(ops);
        typedef Subscriber (*type)(NodeHandle *, SubscribeOptions &);
        auto ptr = (type) dlsym(RTLD_NEXT, "_ZN3ros10NodeHandle9subscribeERNS_16SubscribeOptionsE");
        auto sub_er = ptr(this, ops);
        return sub_er;
    }

} // namespace ros