# MudBoxer
A security 'wrapper' for ROS developed as part of ROS Secure or ROSS project. 

MudBoxer secures your ROS app by:
* Changing `type` of all topics to `std::msgs/String` so only the nodes know the datatype of messages sent/received.
* Encrypting messages of each topic with a key known only to certified nodes publishing/subcribing to that topic.
* Including a checksum within the encrypted message to ensure message intergrity. 

## Install
* Install [Crypt](https://github.com/srinskit/Crypt)
* Build and install MudBoxer(will add more general instructions soon).

## Setup & Run
* Install [Mudboxer](#Install-&-Setup)
* Give the `mudboxer.sh` script the `mudbox` alias.

    Here's mine: `alias mudbox='bash $HOME/Projects/MudBoxer/mudboxer.sh'`
* Setup the following environment variables for each node
    * ROSS_ROOT_CERT="Path to root CA's certificate"
    * ROSS_NODE_NAME="Node's name"
    * ROSS_NODE_KEY="Path to node's private key"
    * ROSS_NODE_PASS="Password to node's private key"
    * ROSS_NODE_CERT=""
    * ROSS_AS_HOST="URN of AuthServer"

    I group mine into a source file like `load` and `. load` before launching that node.
* Build up your `rosrun` command first so that you can still use the auto-complete functionality. Then prefix it with `mudbox `. 

    For example, `rosrun turtlesim turtlesim_node` becomes  `mudbox rosrun turtlesim turtlesim_node`.

Incase your node doesn't run as it used to: mail me the source or atleast the publish and subscribe options so that I can include them into MudBoxer.
