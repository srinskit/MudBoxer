/*
    Made with ❤ by srinSkit.
    Created on 31 May 2018.
*/

#include <dlfcn.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include "ros/ros.h"
#include "Crypt.h"
#include "std_msgs/String.h"
#include "ros/message_deserializer.h"

#define AS_PORT 8000

typedef std_msgs::String ROS_STR;
bool called_init = false, enc = true;
std::string node_name;
Crypt myCrypt;
SecureSock::Client client(&myCrypt);
std::map<std::string, std::vector<boost::shared_ptr<ros::SubscriptionCallbackHelper>>> real_helpers;

void term_mudboxer();

/*
 * Initialize the MudBoxer
 */
void init_mudboxer() {
    if (called_init) return;
    called_init = true;
    printf("\n%s\n", "--- MUDBOXER ---");
    myCrypt.initialize("ROSS");
    node_name = getenv("ROSS_NODE_NAME");
    myCrypt.add_cert("root", getenv("ROSS_ROOT_CERT"));
    myCrypt.load_private_key(getenv("ROSS_NODE_KEY"), getenv("ROSS_NODE_PASS"));
    myCrypt.load_my_cert(getenv("ROSS_NODE_CERT"), "", true);
    client.init();
    if (!client.connect("AuthServer", getenv("ROSS_AS_HOST"), AS_PORT, "root", true)) {
        printf("Could not connect to AuthServer :(\n");
        term_mudboxer();
        exit(EXIT_FAILURE);
    }
}

/*
 * Clean-up after MudBoxer
 */
void term_mudboxer() {
    // Todo: Use a different param to avoid calling terminate multiple times
    if (!called_init)return;
    called_init = false;
    client.close();
    client.terminate();
    myCrypt.terminate();
    printf("%s\n", "\n---   BYE!   ---");
}

/*
 * Wraps message published by ROS node
 * */
typedef boost::function<ros::SerializedMessage(void)> SerFunc;

void mod_msg(const SerFunc &ser_func, SerFunc &new_ser_func, ROS_STR &new_msg) {
    /*
     * Format of final message: topic;payload
     * payload: encrypted serialized message buffer 'buf'
     * format of 'buf'(by ROS): size of 'buf'(4 bytes) + serialized message(num_bytes-4 bytes)
     * rev-engineered ref: ros/serialized_message.h, serializeMessage in ros/serialization.h,
     * Todo: add ref to source files so you don't forget why you did this
     */
    ros::SerializedMessage m2 = ser_func();
    std::string payload(reinterpret_cast<char *>(m2.buf.get()), m2.num_bytes);
    auto topic = new_msg.data.substr(0, new_msg.data.length() - 1);
    if (enc) {
        std::string tmp;
        myCrypt.aes_encrypt(payload, topic, tmp);
        payload = tmp;
    }
    new_msg.data.append(payload);
    new_ser_func = boost::bind(ros::serialization::serializeMessage<ROS_STR>, boost::ref(new_msg));
}

/*
 * Unwraps wrapped message received from wrapped ROS node
 * */
void unmod_msg(const boost::shared_ptr<ROS_STR const> &new_msg, ros::SerializedMessage &m, std::string &topic) {
    std::string str = new_msg->data;
    auto colon_pos = str.find_first_of(';');
    topic = str.substr(0, colon_pos);
    if (enc) {
        std::string tmp;
        myCrypt.aes_decrypt(str.substr(colon_pos + 1), topic, tmp);
        str = topic + ';' + tmp;
    }
    m.num_bytes = new_msg->data.length() - colon_pos - 1;
    /*
     * rev-engineer ref:
     * Todo: add ref to source files so you don't forget why you did this
     */
    m.buf.reset(new uint8_t[m.num_bytes]);
    str.copy(reinterpret_cast<char *>(m.buf.get()), m.num_bytes, colon_pos + 1);
    m.message_start = m.buf.get() + 4;
}

/*
 * Helper function ensure topics are formatted similarly across
 * nodes from different namespaces
 */
std::string standardize_topic(ros::NodeHandle *nh, const std::string &topic) {
    if (topic.length() == 0 || topic[0] == '/')return topic;
    if (nh->getNamespace() == "/") return '/' + topic;
    return nh->getNamespace() + '/' + topic;
}

/*
 * Deliver messages to actual/real callback helpers
 */
void deliver_msg(const boost::shared_ptr<ROS_STR const> &str) {
    ros::SerializedMessage m;
    std::string topic;
    unmod_msg(str, m, topic);
    /*
     * rev-engineer ref:
     * Todo: add ref to source files so you don't forget why you did this
     */
    auto connection_header = boost::make_shared<ros::M_string>();
    ros::SubscriptionCallbackHelperCallParams params;
    for (auto &helper : real_helpers[topic]) {
        auto des = boost::make_shared<ros::MessageDeserializer>(helper, m, connection_header);
        ros::VoidConstPtr msg = des->deserialize();
        params.event = ros::MessageEvent<void const>(msg, des->getConnectionHeader(), ros::Time::now(), false,
                                                     ros::MessageEvent<void const>::CreateFunction());
        helper->call(params);
    }
}

/*
 * Register publisher and modify advertise options
 */
bool mod_pub_ops(ros::NodeHandle *nh, ros::AdvertiseOptions &ops) {
    auto topic = standardize_topic(nh, ops.topic);
    if (topic.empty())
        return false;
    std::stringstream msg;
    msg << "REG_PUB;" << topic;
    client.write(msg.str());
    std::string key;
    client.read(key);
    myCrypt.aes_save_key(topic, key);
    auto qs = ops.queue_size;
    ops.template init<ROS_STR>(topic, qs);
}

/*
 * Register subscriber and modify subscription options
 */
bool mod_sub_ops(ros::NodeHandle *nh, ros::SubscribeOptions &ops) {
    auto topic = standardize_topic(nh, ops.topic);
    if (topic.empty())
        return false;
    std::stringstream msg;
    msg << "REG_SUB;" << topic;
    client.write(msg.str());
    std::string key;
    client.read(key);
    myCrypt.aes_save_key(topic, key);
    real_helpers[topic].push_back(ops.helper);
    auto qs = ops.queue_size;
    ops.helper.reset();
    /*
     * rev-engineer ref:
     * Todo: add ref to source files so you don't forget why you did this
     */
    ops.template init<ROS_STR>(topic, qs, deliver_msg);
}

#define MANGLED_ROS_INIT "_ZN3ros4initERiPPcRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEj"
#define MANGLED_ROS_SHUTDOWN "_ZN3ros8shutdownEv"
#define MANGLED_ROS_PUBLISH "_ZNK3ros9Publisher7publishERKN5boost8functionIFNS_17SerializedMessageEvEEERS3_"
#define MANGLED_ROS_ADVERTISE "_ZN3ros10NodeHandle9advertiseERNS_16AdvertiseOptionsE"
#define MANGLED_ROS_SUBSCRIBE "_ZN3ros10NodeHandle9subscribeERNS_16SubscribeOptionsE"

namespace ros {
    /*
     * Hijack ros::init to initialize MudBoxer
     */
    void init(int &argc, char **argv, const std::string &name, uint32_t options) {
        init_mudboxer();
        typedef void (*type)(int &argc, char **argv, const std::string &name, uint32_t options);
        auto ptr = (type) dlsym(RTLD_NEXT, MANGLED_ROS_INIT);
        ptr(argc, argv, name, options);
    }

    /*
     * Hijack ros::shutdown to terminate MudBoxer
     */
    void shutdown() {
        term_mudboxer();
        typedef void (*type)();
        auto ptr = (type) dlsym(RTLD_NEXT, MANGLED_ROS_SHUTDOWN);
        ptr();
    }

    /*
     * Hijack ros::Publisher::publish to intercept and modify the original message
     */
    void Publisher::publish(const boost::function<SerializedMessage(void)> &ser_func, SerializedMessage &m) const {
        boost::function<SerializedMessage(void)> new_ser_func;
        ROS_STR new_msg;
        new_msg.data = this->getTopic() + ';';
        mod_msg(ser_func, new_ser_func, new_msg);
        typedef void (*type)(const Publisher *, const boost::function<SerializedMessage(void)> &,
                             ros::SerializedMessage &);
        auto ptr = (type) dlsym(RTLD_NEXT, MANGLED_ROS_PUBLISH);
        return ptr(this, new_ser_func, m);
    }

    /*
     * Hijack ros::NodeHandle::advertise to register publisher
     * and change advertisement options
     */
    Publisher NodeHandle::advertise(AdvertiseOptions &ops) {
        mod_pub_ops(this, ops);
        typedef Publisher (*type)(NodeHandle *, AdvertiseOptions &);
        auto ptr = (type) dlsym(RTLD_NEXT, MANGLED_ROS_ADVERTISE);
        auto pub_er = ptr(this, ops);
        // Bypass ROS_ASSERT_MSG at publisher.h line 79
        pub_er.impl_->md5sum_ = "*";
        return pub_er;
    }

    /*
     * Hijack ros::NodeHandle::subscribe to register subscriber
     * and change subscription options
     */
    Subscriber NodeHandle::subscribe(SubscribeOptions &ops) {
        mod_sub_ops(this, ops);
        typedef Subscriber (*type)(NodeHandle *, SubscribeOptions &);
        auto ptr = (type) dlsym(RTLD_NEXT, MANGLED_ROS_SUBSCRIBE);
        auto sub_er = ptr(this, ops);
        return sub_er;
    }

} // namespace ros