package com.github.rosjava.kuka_interface.kuka_action;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
//import org.ros.rosjava.kuka_msgs;

public class PTPServer extends AbstractNodeMain {

    @Override
    public GraphName getDefaultNodeName() {
	return GraphName.of("kuka_actions/ptp_server");
    }

    public void onStart(ConnectedNode connectedNode) {
	connectedNode.newServiceServer("move_ptp", kuka_msgs.MovePTP._TYPE,new ServiceResponseBuilder<kuka_msgs.MovePTPRequest, kuka_msgs.MovePTPResponse>() {
		public void
		    build(kuka_msgs.MovePTPRequest request, kuka_msgs.MovePTPResponse response) {
		    response.setSuccess(true);
		}
	    });
    }
}
