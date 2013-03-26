#ifndef MAIN_H
#define MAIN_H

extern bool running;

void RegisterMarker();
void CreateOSGSphereProxy();
void DeleteVirtualObject(const int & index);

//for Bullet and OSG params
void setWorldOrigin();

//for virtual hands
int	 CreateHand(int lower_left_corn_X, int lower_left_corn_Y) ;
void UpdateAllHands();

//for OSG menu
void AssignPhysics2Osgmenu();
void ResetTextureTransferMode();
void ExecuteAction(const int & val);
void CheckerArInput();
int	 CheckerArModelButtonType(const int & v);

#endif