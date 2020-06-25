#include "turn_control.h"

uint16 induc_ref[4] = {0, 0, 0, 0};

void induc_test(void)
{
  int i,j; 
  int ad_test[4]={0,0,0,0};
  for(j=0;j<3;j++)
  {
    getl_once();
    ad_test[0] += l_h_1;
    ad_test[1] += l_h_2;   
    ad_test[2] += l_s_1; 
    ad_test[3] += l_s_2;
  }
  for(i=0;i<4;i++)
  {
    ad_test[i]=ad_test[i]/3;  
    // if(ad_test[i]>Induc_Ref[2*i])
    // {
    //   Induc_Ref[2*i]=ad_test[i];
    // }
  }
}

void direction_control(void)
{
    int i, j;
    int ad[4] = {0, 0, 0, 0};
    for(i=0;i<3;i++)
    {
        getl_once();
        ad[0] += l_h_1;
        ad[1] += l_h_2;
        ad[2] += l_s_1;
        ad[3] += l_s_2;
    }
    for(i=0;i<4;i++)
        ad[i] = ad[i]/3;
}