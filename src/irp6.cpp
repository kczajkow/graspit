
#include "irp6.h"
#include "debug.h"

int
IRp6::invKinematics(const transf& endTranLocal,double* dofVals,int)
{
    DBGA("IRp6::invKinematics");
    DBGA(endTranLocal.translation());
    DBGA(endTranLocal.affine());
	for (int i=0; i<numDOF; ++i)
		DBGA(dofVec[i]->getVal());

  	// Pomocnicze
	transf endTran = endTranLocal * base->getTran().inverse();
	mat3 affine = endTran.affine();
	vec3 translation = endTran.translation();
	double local_desired_joints[6];
	double local_current_joints[6];
	const double a2 = 455;
	const double a3 = 670;
	const double d5 = 190;

	for (int i=0; i<numDOF; ++i)
		local_current_joints[i] = dofVec[i]->getVal();
	//local_current_joints[2] += local_current_joints[1];
	//local_current_joints[3] += local_current_joints[2];
	local_current_joints[1] -= 1.542;
	local_current_joints[4] += 4.712;

	// Stale
	const double EPS = 1e-10;

	// Zmienne pomocnicze.
	double Nx, Ox, Ax, Px;
	double Ny, Oy, Ay, Py;
	double Nz, Oz, Az, Pz;
	double s0, c0, s1, c1, s3, c3, s4, c4;
	double E, F, K, ro, G, H;
	double t5, t_ok;

	// Przepisanie zmiennych.
	Nx = affine.element(0,0);
	Ny = affine.element(1,0);
	Nz = affine.element(2,0);
	Ox = affine.element(0,1);
	Oy = affine.element(1,1);
	Oz = affine.element(2,1);
	Ax = affine.element(0,2);
	Ay = affine.element(1,2);
	Az = affine.element(2,2);
	Px = translation[0];
	Py = translation[1];
	Pz = translation[2];

	//  Wyliczenie Theta1.
	local_desired_joints[0] = atan2(Py, Px);
	s0 = sin(local_desired_joints[0]);
	c0 = cos(local_desired_joints[0]);

	// Wyliczenie Theta5.
	c4 = Ay * c0 - Ax * s0;
	// Sprawdzenie bledow numerycznych.
	if (fabs(c4 * c4 - 1) > EPS)
		s4 = sqrt(1 - c4 * c4);
	else
		s4 = 0;

	double cj_tmp;
	double dj_translation;
	// Sprawdzenie rozwiazania.
	if (local_current_joints[4] > M_PI) {
		cj_tmp = local_current_joints[4] - 2*M_PI;
		dj_translation = 2*M_PI;
	} else if (local_current_joints[4] < -M_PI) {
		cj_tmp = local_current_joints[4] + 2*M_PI;
		dj_translation = -2*M_PI;
	} else {
		cj_tmp = local_current_joints[4];
		dj_translation = 0.0;
	}

	// Niejednoznacznosc - uzywamy rozwiazanie blizsze poprzedniemu.
	if (cj_tmp > 0)
		local_desired_joints[4] = atan2(s4, c4);
	else
		local_desired_joints[4] = atan2(-s4, c4);

	// Dodanie przesuniecia.
	local_desired_joints[4] += dj_translation;

	// Wyliczenie Theta4 i Theta6.
	if (fabs(s4) < EPS) {
		printf("Osobliwosc\n");
		// W przypadku osobliwosci katowi theta4 przypisywana wartosc poprzednia.
		local_desired_joints[3] = local_current_joints[3];
		t5 = atan2(c0 * Nx + s0 * Ny, c0 * Ox + s0 * Oy);

		// Sprawdzenie warunkow.
		t_ok = t5 + local_desired_joints[3];
		if (fabs(t_ok - local_current_joints[5])
				> fabs(t5 - M_PI + local_desired_joints[3] - (local_current_joints[5])))
			t_ok = t5 - M_PI + local_desired_joints[3];
		if (fabs(t_ok - local_current_joints[5])
				> fabs(t5 + M_PI + local_desired_joints[3] - (local_current_joints[5])))
			t_ok = t5 + M_PI + local_desired_joints[3];

		if (fabs(t_ok - local_current_joints[5]) > fabs(t5 - 2*M_PI + local_desired_joints[3]
				- (local_current_joints[5])))
			t_ok = t5 - 2*M_PI + local_desired_joints[3];
		if (fabs(t_ok - local_current_joints[5]) > fabs(t5 + 2*M_PI + local_desired_joints[3]
				- (local_current_joints[5])))
			t_ok = t5 + 2*M_PI + local_desired_joints[3];

		if (fabs(t_ok - local_current_joints[5]) > fabs(t5 - local_desired_joints[3] - (local_current_joints[5])))
			t_ok = t5 - local_desired_joints[3];
		if (fabs(t_ok - local_current_joints[5])
				> fabs(t5 - M_PI - local_desired_joints[3] - (local_current_joints[5])))
			t_ok = t5 - M_PI - local_desired_joints[3];
		if (fabs(t_ok - local_current_joints[5])
				> fabs(t5 + M_PI - local_desired_joints[3] - (local_current_joints[5])))
			t_ok = t5 + M_PI - local_desired_joints[3];

		if (fabs(t_ok - local_current_joints[5]) > fabs(t5 - 2*M_PI - local_desired_joints[3]
				- (local_current_joints[5])))
			t_ok = t5 - 2*M_PI - local_desired_joints[3];
		if (fabs(t_ok - local_current_joints[5]) > fabs(t5 + 2*M_PI - local_desired_joints[3]
				- (local_current_joints[5])))
			t_ok = t5 + 2*M_PI - local_desired_joints[3];

		local_desired_joints[5] = t_ok;
	} else {
		t5 = atan2(-s0 * Ox + c0 * Oy, s0 * Nx - c0 * Ny);
		t_ok = t5;

		// Sprawdzenie warunkow.
		if (fabs(t_ok - local_current_joints[5]) > fabs(t5 - M_PI - (local_current_joints[5])))
			t_ok = t5 - M_PI;
		if (fabs(t_ok - local_current_joints[5]) > fabs(t5 + M_PI - (local_current_joints[5])))
			t_ok = t5 + M_PI;

		local_desired_joints[5] = t_ok;
		t_ok = atan2(c0 * Ax + s0 * Ay, Az);

		if (fabs(t_ok - local_current_joints[3]) > fabs(t_ok - M_PI - (local_current_joints[3])))
			t_ok = t_ok - M_PI;
		if (fabs(t_ok - local_current_joints[3]) > fabs(t_ok + M_PI - (local_current_joints[3])))
			t_ok = t_ok + M_PI;
		local_desired_joints[3] = t_ok;
	}//: else

	// Wyliczenie Theta2.
	c3 = cos(local_desired_joints[3]);
	s3 = sin(local_desired_joints[3]);

	E = c0 * Px + s0 * Py - c3 * d5;
	F = -Pz - s3 * d5;
	G = 2* E * a2;
	H = 2* F * a2;
	K = E * E + F * F + a2 * a2 - a3 * a3;
	ro = sqrt(G * G + H * H);

	local_desired_joints[1] = atan2(K / ro, sqrt(1 - ((K * K) / (ro * ro)))) - atan2(G, H);

	// Wyliczenie Theta3.
	s1 = sin(local_desired_joints[1]);
	c1 = cos(local_desired_joints[1]);
	local_desired_joints[2] = atan2(F - a2 * s1, E - a2 * c1);

	local_desired_joints[1] += 1.542;
	local_desired_joints[4] -= 4.712;
	//local_current_joints[3] -= local_current_joints[2];
	//local_current_joints[2] -= local_current_joints[1];

	for (int i=0; i<numDOF; ++i)
		dofVals[i] = local_desired_joints[i];

	DBGA("Obliczone:");
	for (int i=0; i<numDOF; ++i)
		DBGA(dofVals[i]);

  if (false)
	  return FAILURE;
  return SUCCESS;
}