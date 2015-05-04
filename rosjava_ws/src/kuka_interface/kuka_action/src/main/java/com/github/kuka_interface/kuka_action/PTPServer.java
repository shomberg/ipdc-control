package com.github.rosjava.kuka_interface.kuka_action;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import no.hials.crosscom.CrossComClient;
import no.hials.crosscom.KRL.structs.KRLPos;

public class PTPServer extends AbstractNodeMain {

    private static final double XTHRESH = .1;
    private static final double YTHRESH = .1;
    private static final double ZTHRESH = .1;
    private static final double ATHRESH = .1;
    private static final double BTHRESH = .1;
    private static final double CTHRESH = .1;
    private CrossComClient client;

    @Override
    public GraphName getDefaultNodeName() {
	return GraphName.of("kuka_actions/ptp_server");
    }

    public void onStart(ConnectedNode connectedNode) {
	try{
	    client = new CrossComClient("172.31.1.147", 7000);
	} catch(Exception e){
	    e.printStackTrace();
	}
	KRLPos pos = new KRLPos("MYPOS");
	KRLE6Pos pos_act = new KRLE6Pos("$POS_ACT");
	connectedNode.newServiceServer("move_ptp", kuka_msgs.MovePTP._TYPE,new ServiceResponseBuilder<kuka_msgs.MovePTPRequest, kuka_msgs.MovePTPResponse>() {
		public void build(kuka_msgs.MovePTPRequest request, kuka_msgs.MovePTPResponse response) {
		    pos.setX(request.getX());
		    pos.setY(request.getY());
		    pos.setZ(request.getZ());
		    pos.setA(request.getA());
		    pos.setB(request.getB());
		    pos.setC(request.getC());
		    long start = System.currentTimeMillis();
		    while(System.currentTimeMillis() - start < TIMEOUT){
			client.readVariable(pos_act);
			double xOff = Math.abs(pos_act.getX() - request.getX());
			double yOff = Math.abs(pos_act.getY() - request.getY());
			double zOff = Math.abs(pos_act.getZ() - request.getZ());
			double aOff = Math.abs(pos_act.getA() - request.getA());
			double bOff = Math.abs(pos_act.getB() - request.getB());
			double cOff = Math.abs(pos_act.getC() - request.getC());
			if(xOff < XTHRESH && yOff < YTHRESH && zOff < ZTHRESH
			   && aOff < ATHRESH && bOff < BTHRESH && cOff < CTHRESH){
			    response.setSuccess(true);
			    return;
			}
		    }
		    response.setSuccess(false);
		}
	    });
    }
}
