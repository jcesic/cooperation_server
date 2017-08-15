#include <iostream>
#include <stdio.h>
#include <SE3Mappings.h>

using namespace std;

int main(){
	TranVector t,angles;
	RotMat R;
	SE3Mat RS;
	StateMat adj;
	StateVector tS;
	RS.setIdentity();
	tS.setZero();

	t << -4.6023060515737190812046720, -66.5807444299353790029272204, 2236.7498111652130319271236658;
	R << 0.9997864504998164036919661, 0.0150988694949337892703412, -0.0141094839364552162436217, -0.0150924798097180502126635, 0.9998859456478227381026613, 0.0005592397987282309110985, 0.0141163185771426592796107, -0.0003461732719122382435570, 0.9999002998868908864693594;
	angles << -0.0141099543318687729864802, -0.0005592398278785304897853, -0.0150930551970604003020338;

	RS.block(0,0,3,3)=R;
	RS.block(0,3,3,1)=t;
	tS.block(0,0,3,1)=t;
	tS.block(3,0,3,1)=angles;


	SE3Mappings SEMap;

	cout << "adj" << endl << SEMap.adj(tS) << endl;
	cout << "Ad" << endl << SEMap.Ad(RS) << endl;
	cout << "exp" << endl << SEMap.exp(tS) << endl;
	cout << "hat" << endl << SEMap.SE3x(t) << endl;
	cout << "invHat" << endl << SEMap.invHat(R) << endl;
	cout << "log" << endl << SEMap.log(RS) << endl;
	cout << "logV" << endl << SEMap.logV(RS) << endl;
	adj = SEMap.adj(tS);
	cout << "PhiG" << endl << SEMap.PhiG(adj) << endl;
	cout << "angle2axis" << endl << SEMap.angle2axis(angles) << endl;

}
