#ifndef CODE_MY_FUZZY_H_
#define CODE_MY_FUZZY_H_

#define range 2

extern float Range_PError[5];
extern float Range_DError[5];

extern float Output_tmp_P_left[5];
extern float Output_tmp_D_left[5];
extern float Output_tmp_P_right[5];
extern float Output_tmp_D_right[5];
extern float Output_tmp_D[5];

extern float fuzzy_P[range][5];
extern float fuzzy_D[range][5];

void GetFuzzyPD(float Error,float DError);

#endif /* CODE_MY_FUZZY_H_ */
