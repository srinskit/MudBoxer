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
    if (!client.connect("AuthServer", "localhost", 8000)) {
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

void myAdvert(ros::AdvertiseOptions &ops) {
    printf("ADVERTISING %s %s\n", ops.topic.c_str(), ops.datatype.c_str());
    std::stringstream msg;
    msg << "Hello AS";
    client.write(msg.str());
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

    Publisher NodeHandle::advertise(AdvertiseOptions &ops) {
        myAdvert(ops);
        typedef Publisher (*type)(NodeHandle *, AdvertiseOptions &);
        auto ptr = (type) dlsym(RTLD_NEXT, "_ZN3ros10NodeHandle9advertiseERNS_16AdvertiseOptionsE");
        return ptr(this, ops);
    }

    Subscriber NodeHandle::subscribe(SubscribeOptions &ops) {
        printf("SUBSCRIBING to %s\n", ops.topic.c_str());
        typedef Subscriber (*type)(NodeHandle *, SubscribeOptions &);
        auto ptr = (type) dlsym(RTLD_NEXT, "_ZN3ros10NodeHandle9subscribeERNS_16SubscribeOptionsE");
        return ptr(this, ops);
    }
} // namespace ros