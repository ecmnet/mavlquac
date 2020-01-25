package com.comino.mavlquac.odometry.pose;

import java.awt.Color;
import java.awt.Graphics;
import java.util.concurrent.TimeUnit;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_msp_vision;
import org.mavlink.messages.lquac.msg_vision_position_estimate;
import org.mavlink.messages.lquac.msg_vision_speed_estimate;

import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavlquac.video.IVisualStreamHandler;
import com.comino.mavmap.utils.MSP3DUtils;
import com.comino.mavodometry.librealsense.t265.boofcv.StreamRealSenseT265Pose;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.legacy.ExecutorService;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.GeoTuple3D_F64;
import georegression.struct.se.Se3_F32;
import georegression.struct.se.Se3_F64;

public class StreamRealSensePose {

	private static final float OFFSET = 0.03f;

	public static final int  GROUNDTRUTH_MODE   = 1;
	public static final int  LPOS_MODE          = 2;

	private static final int WIDTH  = 320;
	private static final int HEIGHT = 240;

	private final msg_vision_position_estimate sms = new msg_vision_position_estimate(1,2);

	private final msg_msp_vision msg = new msg_msp_vision(2,1);

	private StreamRealSenseT265Pose t265;
	private IMAVMSPController control;
	private DataModel model;

	private Se3_F64    to_ned = new Se3_F64();
	private Se3_F64    ned    = new Se3_F64();

	private boolean is_initialized = false;

	private float initial_yaw;

	private final Color	bgColor = new Color(128,128,128,130);

	private double[] visAttitude     = new double[3];

	public <T> StreamRealSensePose(IMAVMSPController control, int mode, IVisualStreamHandler<GrayU8> stream) {

		this.control = control;
		this.model = control.getCurrentModel();

		final msg_msp_vision msg = new msg_msp_vision(2,1);


		// reset vision when armed
		control.getStatusManager().addListener( Status.MSP_ARMED, (n) -> {
			if(n.isStatus(Status.MSP_ARMED)) {
				reset();
			}
		});

		//reset vision when GPOS gets valid
		control.getStatusManager().addListener(Status.MSP_GPOS_VALID, (n) -> {
			if((n.isStatus(Status.MSP_GPOS_VALID)))
				reset();
		});

		// reset after 30 seconds (Workaround for Compass ramp up)
		ExecutorService.get().schedule(() -> {
			is_initialized = true;
			reset();
		}, 40, TimeUnit.SECONDS);

		stream.registerOverlayListener(ctx -> {
			overlayFeatures(ctx);
		});


		t265 = new StreamRealSenseT265Pose(StreamRealSenseT265Pose.POS_FOREWARD,WIDTH,HEIGHT,(tms, raw, p, s, left, right) ->  {
//
//			if(!is_initialized)
//				return;


			// correction mounting offset
			// TODO: also pitch and roll compensation via rotation matrix
			p.T.x = p.T.x - (float)Math.cos(model.attitude.y - initial_yaw)*OFFSET;
			p.T.y = p.T.y - (float)Math.sin(model.attitude.y - initial_yaw)*OFFSET;

			// transform p -> ned
			p.concat(to_ned, ned);


			switch(mode) {

			case GROUNDTRUTH_MODE:

				// set ground truth
				model.vision.gx =  (float)ned.getX();
				model.vision.gy =  (float)ned.getY();
				model.vision.gz =  (float)ned.getZ();

				break;

			case LPOS_MODE:

                stream.addToStream(left, model, tms);

                ConvertRotation3D_F64.matrixToEuler(ned.R, EulerType.XYZ, visAttitude);

				publishPX4Vision(ned,tms);
				publisMSPVision(ned,tms);


				break;

			}
		});

	}

	public void reset() {
//		t265.reset();
		MSP3DUtils.convertModelToSe3_F64(model, to_ned);
		initial_yaw = model.attitude.y;
		this.control.writeLogMessage(new LogMessage("[slm] Pose estimation reset", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
	}

	public void start() {
		t265.start();
		System.out.println("Starting T265....");
		MSP3DUtils.convertModelToSe3_F64(model, to_ned);
		initial_yaw = model.attitude.y;
		this.control.writeLogMessage(new LogMessage("[slm] Pose estimation init", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
		t265.printDeviceInfo();
	}

	public void stop() {
		t265.stop();
	}

	private void overlayFeatures(Graphics ctx) {

		ctx.setColor(bgColor);
		ctx.fillRect(5, 5, WIDTH-10, 21);
		ctx.setColor(Color.white);

		if(!Float.isNaN(model.sys.t_armed_ms) && model.sys.isStatus(Status.MSP_ARMED))
			ctx.drawString(String.format("%.1f sec",model.sys.t_armed_ms/1000), 20, 20);

		if(model.msg.text != null && (model.sys.getSynchronizedPX4Time_us()-model.msg.tms) < 1000000)
			ctx.drawString(model.msg.text, 10, HEIGHT-5);

	}

	private void publishPX4Vision(Se3_F64 pose, long tms) {

		sms.usec = tms;

		sms.x = (float) pose.T.x;
		sms.y = (float) pose.T.y;
		sms.z = (float) pose.T.z;

		sms.roll  = (float)visAttitude[0];
		sms.pitch = (float)visAttitude[1];
		sms.yaw   = (float)visAttitude[2];

		sms.covariance[0] = Float.NaN;

		control.sendMAVLinkMessage(sms);
//
//		smv.usec = tms;
//
//		smv.x = (float) speed.x;
//		smv.y = (float) speed.y;
//		smv.z = (float) speed.z;
//
//		control.sendMAVLinkMessage(smv);

		model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, true);

	}

	private void publisMSPVision(Se3_F64 pose, long tms) {

			msg.x =  (float) pose.T.x;
			msg.y =  (float) pose.T.y;
			msg.z =  (float) pose.T.z;


            msg.h = MSPMathUtils.fromRad((float)visAttitude[2]);

            msg.r   = (float)visAttitude[0];
    		msg.p   = (float)visAttitude[1];
    		msg.y   = (float)visAttitude[2];


			msg.quality = 100;
			msg.fps = 33;
			//	msg.tms = (long)estTimeDepth_us;
			msg.tms = tms;

			control.sendMAVLinkMessage(msg);


	}

}
