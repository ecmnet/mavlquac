package com.comino.mavlquac.odometry.estimators;

import com.comino.mavlquac.odometry.detectors.IObstacleDetector;
import com.comino.mavlquac.video.IVisualStreamHandler;
import com.comino.mavodometry.librealsense.r200.vio.odometry.MAVDepthVisualOdometry;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;

public interface IPositionEstimator {

	void registerDetector(IObstacleDetector detector);

	void registerStreams(IVisualStreamHandler stream);

	void start();

	void stop();

	boolean isRunning();

	void reset();

	void enableDetectors( boolean enable);

	MAVDepthVisualOdometry<GrayU8,GrayU16> getOdometry();

}