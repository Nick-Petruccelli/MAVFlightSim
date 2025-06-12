#include "../inc/mav_dynamics_model.hpp"
#include <cmath>

MAVDynamicsModel::MAVDynamicsModel(float mass, float Jx, float Jy, float Jz, float Jxz, Vector3 init_pos, Vector3 init_rot, Vector3 init_vel, Vector3 init_ang_vel){
	m_mass = mass;

	m_Jx = Jx;
	m_Jy = Jy;
	m_Jz = Jz;
	m_Jxz = Jxz;

	m_gamma = m_Jx * m_Jz - m_Jxz * m_Jxz;
	m_gamma1 = (m_Jxz * (m_Jx - m_Jy + m_Jz)) / m_gamma;
	m_gamma2 = (m_Jz * (m_Jz - m_Jy) + m_Jxz * m_Jxz) / m_gamma;
	m_gamma3 = m_Jz / m_gamma;
	m_gamma4 = m_Jxz / m_gamma;
	m_gamma5 = (m_Jz - m_Jx) / m_Jy;
	m_gamma6 = m_Jxz / m_Jy;
	m_gamma7 = ((m_Jx - m_Jy) * m_Jx + m_Jxz * m_Jxz) / m_gamma;
	m_gamma8 = m_Jx / m_gamma;

	m_pn = init_pos.x;
	m_pe = init_pos.z;
	m_pd = -init_pos.y;
	
	m_u = init_vel.x;
	m_v = init_vel.z;
	m_w = -init_vel.y;

	m_p = init_ang_vel.x;
	m_q = init_ang_vel.z;
	m_r = -init_ang_vel.y;
	
	float psi = init_rot.x;
	float phi = init_rot.z;
	float theta = -init_rot.y;
	m_e0 = cos(psi/2.0)*cos(theta/2.0)*cos(phi/2.0) + sin(psi/2.0)*sin(theta/2.0)*sin(phi/2.0);
	m_ex = cos(psi/2.0)*cos(theta/2.0)*sin(phi/2.0) - sin(psi/2.0)*sin(theta/2.0)*cos(phi/2.0);
	m_ey = cos(psi/2.0)*sin(theta/2.0)*cos(phi/2.0) + sin(psi/2.0)*cos(theta/2.0)*sin(phi/2.0);
	m_ez = sin(psi/2.0)*cos(theta/2.0)*cos(phi/2.0) - cos(psi/2.0)*sin(theta/2.0)*sin(phi/2.0);
}

Vector3 MAVDynamicsModel::get_pos(){
	Vector3 out;
	out.x = m_pn;
	out.y = -m_pd;
	out.z = m_pe;
	return out;
}

Vector3 MAVDynamicsModel::get_rot(){
	Vector3 out;
	out.x = atan2f(2.0f * (m_e0 * m_ex + m_ey * m_ez), m_e0 * m_e0 + m_ez * m_ez - m_ex * m_ex - m_ey * m_ey);
	out.y = asinf(2 * (m_e0 * m_ey - m_ex * m_ez));
	out.z = atan2f(2.0f * (m_e0 * m_ez + m_ex * m_ey), m_e0 * m_e0 + m_ex * m_ex - m_ey * m_ey - m_ez * m_ez);
	return out;
}

void MAVDynamicsModel::apply_force(float dt, Vector3 translation, Vector3 rotation){
	float fx = translation.x;
	float fy = translation.y;
	float fz = translation.z;
	float Mx = rotation.x;
	float My = rotation.y;
	float Mz = rotation.z;

	euler_step(dt, fx, fy, fz, Mx, My, Mz);
}

void MAVDynamicsModel::euler_step(float dt, float fx, float fy, float fz, float Mx, float My, float Mz){
	float pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot, e0_dot, ex_dot, ey_dot, ez_dot, p_dot, q_dot, r_dot;

	pn_dot =
		(m_e0 * m_e0 + m_ex * m_ex - m_ey * m_ey - m_ez * m_ez) * m_u +
		2 * (m_ex * m_ey - m_e0 * m_ez) * m_v +
		2 * (m_ex * m_ez +  m_e0 * m_ey) * m_w;
	pe_dot =
		2 * (m_ex * m_ey + m_e0 * m_ez) * m_u +
		(m_e0 * m_e0 - m_ex * m_ex + m_ey * m_ey - m_ez * m_ez) * m_v +
		2 * (m_ey * m_ez - m_e0 * m_ex) * m_w;
	pd_dot =
		2 * (m_ex * m_ez -  m_e0 * m_ey) * m_u +
		2 * (m_ey * m_ez + m_e0 * m_ex) * m_v +
		(m_e0 * m_e0 - m_ex * m_ex - m_ey * m_ey + m_ez * m_ez) * m_w;

	u_dot = (m_r * m_v - m_q * m_w) + (fx / m_mass);
	v_dot = (m_p * m_w - m_r * m_u) + (fy / m_mass);
	w_dot = (m_q * m_u - m_p * m_v) + (fz / m_mass);

	e0_dot = (-m_p * m_ex + -m_q * m_ey + -m_r * m_ez) / 2.0f;
	ex_dot = (m_p * m_e0 + m_r * m_ey + -m_q * m_ez) / 2.0f;
	ey_dot = (m_q * m_e0 + -m_r * m_ex + m_p * m_ez) / 2.0f;
	ez_dot = (m_r * m_e0 + m_q * m_ex + -m_p * m_ey) / 2.0f;

	p_dot = (m_gamma1 * m_p * m_q - m_gamma2 * m_q * m_r) + (m_gamma3 * Mx + m_gamma4 * Mz);
	q_dot = (m_gamma5 * m_p * m_r - m_gamma6 * (m_p * m_p - m_r * m_r)) + (My / m_Jy);
	r_dot = (m_gamma7 * m_p * m_q - m_gamma1 * m_q * m_r) + (m_gamma4 * Mx + m_gamma8 * Mz);

	m_pn += dt * pn_dot;
	m_pe += dt * pe_dot;
	m_pd += dt * pd_dot;

	m_u += dt * u_dot;
	m_v += dt * v_dot;
	m_w += dt * w_dot;

	m_e0 += dt * e0_dot;
	m_ex += dt * ex_dot;
	m_ey += dt * ey_dot;
	m_ez += dt * ez_dot;

	m_p += dt * p_dot;
	m_q += dt * q_dot;
	m_r += dt * r_dot;
}
