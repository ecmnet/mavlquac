package com.comino.mavlquac.flighttask.trajectory;

import com.comino.mavlquac.flighttask.types.VehicleState4D_F32;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point3D_F32;

public class Rapid4DTrajectoryGenerator {
	
	private final float MIN_ACC		   = 0.0f;
	private final float MAX_ACC		   = 0.5f;
	private final float MAX_BODY_RATE  = 2;
	private final float TIME_STEP      = 0.02f;
	
	private final SingleAxisTrajectory _axis[] = new SingleAxisTrajectory[4];
	
	private Point3D_F32 _gravity;
	private float       _tf;

	private final Point3D_F32 _tmp1 = new Point3D_F32();
	private final Point3D_F32 _tmp2 = new Point3D_F32();
	
	
	public Rapid4DTrajectoryGenerator() {
		super();
		
		for(int i=0;i<4;i++) {
			_axis[i] = new SingleAxisTrajectory();
		}
		this._gravity = new Point3D_F32();
	}
	

	public Rapid4DTrajectoryGenerator(Point3D_F32 gravity) {
		super();
		for(int i=0;i<4;i++) {
			_axis[i] = new SingleAxisTrajectory();
		}
		this._gravity = gravity;
	}
	
	public void setInitialState(VehicleState4D_F32 current) {
		reset();
		for(int i=0;i<4;i++) {
			_axis[i].setInitialState(current.pos().getIdx(i),current.vel().getIdx(i),current.acc().getIdx(i));
		}
	}
	
	public void setGoal(GeoTuple4D_F32<?> p, GeoTuple4D_F32<?> v) {
		for(int i=0;i<4;i++) {
			_axis[i].setGoalPosition(p.getIdx(i));
			_axis[i].setGoalVelocity(v.getIdx(i));
			_axis[i].setGoalAcceleration(0);
		}	
	}
	
	public void setGoal(GeoTuple4D_F32<?> p) {
		for(int i=0;i<4;i++) {
			_axis[i].setGoalPosition(p.getIdx(i));
			_axis[i].setGoalVelocity(0);
			_axis[i].setGoalAcceleration(0);
		}	
	}
	
	public float  generate(float timeToFinish) {
		_tf = timeToFinish;
		for(int i=0;i<4;i++)
			_axis[i].generateTrajectory(_tf);
		return (float) _tf;
	}
	
	public void getStateAt(float t, VehicleState4D_F32 state) {
		for(int i=0;i<4;i++) {
			state.acc().setIdx(i, _axis[i].getAcceleration(t));
			state.vel().setIdx(i, _axis[i].getVelocity(t));
			state.pos().setIdx(i, _axis[i].getPosition(t));
		}
	}
	
	public float geTimeToFinish() {
		return (float)_tf;
	}
	
	public void reset() {
		for(int i=0;i<4;i++)
			_axis[i].reset();
		_tf = 0;
	}
	
	


}
