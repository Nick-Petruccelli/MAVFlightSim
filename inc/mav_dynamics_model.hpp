#pragma once
#include <raylib.h>
#include <raymath.h>

class MAVDynamicsModel {
public:
	MAVDynamicsModel(float mass, float Jx, float Jy, float Jz, float Jxz, Vector3 init_pos, Vector3 init_rot, Vector3 init_vel, Vector3 init_ang_vel);
	void apply_force(float dt, Vector3 translational, Vector3 rotational);
	Vector3 get_pos();
	Vector3 get_rot();
private:
	void euler_step(float dt, float fx, float fy, float fz, float Mx, float My, float Mz);

	float m_mass;
	float m_Jx;
	float m_Jy;
	float m_Jz;
	float m_Jxz;
	float m_gamma;
	float m_gamma1;
	float m_gamma2;
	float m_gamma3;
	float m_gamma4;
	float m_gamma5;
	float m_gamma6;
	float m_gamma7;
	float m_gamma8;

	float m_pn;
	float m_pe;
	float m_pd;

	float m_u;
	float m_v;
	float m_w;

	float m_e0;
	float m_ex;
	float m_ey;
	float m_ez;

	float m_p;
	float m_q;
	float m_r;
};
