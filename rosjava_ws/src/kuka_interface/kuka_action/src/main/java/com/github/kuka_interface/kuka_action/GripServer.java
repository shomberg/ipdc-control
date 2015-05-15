package com.github.rosjava.kuka_interface.kuka_action;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import no.hials.crosscom.CrossComClient;
import no.hials.crosscom.KRL.KRLBool;

public class GripServer extends AbstractNodeMain {

    private CrossComClient client;
    private KRLBool grip;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("kuka_actions/grip_server");
    }

    public void onStart(ConnectedNode connectedNode) {
	try{
	    client = new CrossComClient("172.31.1.147", 7000);
	} catch(Exception e){
	    System.out.println("Error: connection refused");
	}
	grip = new KRLBool("GRIP");
	connectedNode.newServiceServer("set_grip", kuka_msgs.SetGrip._TYPE,new ServiceResponseBuilder<kuka_msgs.SetGripRequest, kuka_msgs.SetGripResponse>() {
		public void build(kuka_msgs.SetGripRequest request, kuka_msgs.SetGripResponse response) {
		    grip.setValue(request.getGrip());
		    System.out.println(grip);
		    try{
			client.writeVariable(grip);
		    } catch(Exception e){
			e.printStackTrace();
		    }
		    response.setSuccess(true);
		}
	    });
    }
}
