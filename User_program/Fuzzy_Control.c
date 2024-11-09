#include "Fuzzy_Control.h"

// 定義模糊集合大小
#define ZO  0     // 零
#define PS1 1500  // 小正1
#define PS2 3000  // 小正2
#define PM  4500  // 中正
#define PB  5950  // 大正

extern Fuzzy_Control fuzzy_spd;
extern Fuzzy_Control fuzzy_curr;

// 模糊規則表 1x5
const float fuzzyRuleTable[5] = {ZO, PS1, PS2, PM, PB};

// 隸屬度函數
float getMembership(float x, float left, float middle, float right) {
    if (x <= left || x >= right) return 0.0f;
    if (x == middle) return 1.0f;
    if (x < middle) return (x - left) / (middle - left);
    return (right - x) / (right - middle);
}


// void Fuzzy_Controller(p_Fuzzy_Control pV) {
//     float E = pV->Error * pV->KE;
//     float EC = (pV->Error - pV->LastError) * pV->KEC;
//     float U = 0;
//     float numerator = 0;
//     float denominator = 0;
//     int i, j;  // 在循環外部聲明
//     float eMembership = 0;
//     float ecMembership = 0;
//     float membershipProduct = 0;
    
//     // 計算隸屬度
//     float eNB = getMembership(E, -3, -3, -2);
//     float eNM = getMembership(E, -3, -2, -1);
//     float eNS = getMembership(E, -2, -1, 0);
//     float eZO = getMembership(E, -1, 0, 1);
//     float ePS = getMembership(E, 0, 1, 2);
//     float ePM = getMembership(E, 1, 2, 3);
//     float ePB = getMembership(E, 2, 3, 3);
    
//     float ecNB = getMembership(EC, -3, -3, -2);
//     float ecNM = getMembership(EC, -3, -2, -1);
//     float ecNS = getMembership(EC, -2, -1, 0);
//     float ecZO = getMembership(EC, -1, 0, 1);
//     float ecPS = getMembership(EC, 0, 1, 2);
//     float ecPM = getMembership(EC, 1, 2, 3);
//     float ecPB = getMembership(EC, 2, 3, 3);
    
//     // 解模糊化
//     for (i = 0; i < 7; i++) {  // 移除 int 聲明
//         for (j = 0; j < 7; j++) {  // 移除 int 聲明
//             eMembership = 0;
//             ecMembership = 0;
            
//             switch(i) {
//                 case 0: eMembership = eNB; break;
//                 case 1: eMembership = eNM; break;
//                 case 2: eMembership = eNS; break;
//                 case 3: eMembership = eZO; break;
//                 case 4: eMembership = ePS; break;
//                 case 5: eMembership = ePM; break;
//                 case 6: eMembership = ePB; break;
//             }
            
//             switch(j) {
//                 case 0: ecMembership = ecNB; break;
//                 case 1: ecMembership = ecNM; break;
//                 case 2: ecMembership = ecNS; break;
//                 case 3: ecMembership = ecZO; break;
//                 case 4: ecMembership = ecPS; break;
//                 case 5: ecMembership = ecPM; break;
//                 case 6: ecMembership = ecPB; break;
//             }
            
//             membershipProduct = eMembership * ecMembership;
//             numerator += membershipProduct * fuzzyRuleTable[i][j];
//             denominator += membershipProduct;
//         }
//     }
    
//     if (denominator != 0) {
//         U = (numerator / denominator) * pV->KU;
//     }
    
//     // 限制輸出範圍
//     if (U > pV->Umax) U = pV->Umax;
//     if (U < pV->Umin) U = pV->Umin;
    
//     pV->Output = U;
//     pV->LastError = pV->Error;
// }


void Fuzzy_Controller(p_Fuzzy_Control pV) {
    float E = pV->Error * pV->KE;  // 誤差
    float U = 0;
    float numerator = 0;
    float denominator = 0;
    
    // 計算隸屬度
    float eZO = getMembership(E, 0, 0, 1500);     // 零區間
    float ePS1 = getMembership(E, 0, 1500, 3000);     // 小正1區間
    float ePS2 = getMembership(E, 1500, 3000, 4500);  // 小正2區間
    float ePM = getMembership(E, 3000, 4500, 5950);   // 中正區間
    float ePB = getMembership(E, 4500, 5950, 5950);   // 大正區間
    
    // 計算加權平均
    numerator = eZO * fuzzyRuleTable[0] +
                ePS1 * fuzzyRuleTable[1] +
                ePS2 * fuzzyRuleTable[2] +
                ePM * fuzzyRuleTable[3] +
                ePB * fuzzyRuleTable[4];
                
    denominator = eZO + ePS1 + ePS2 + ePM + ePB;
    
    // 解模糊化
    if (denominator != 0) {
        U = (numerator / denominator) * pV->KU;
    }
    
    // 限制輸出範圍
    if (U > pV->Umax) U = pV->Umax;
    if (U < pV->Umin) U = pV->Umin;
    
    pV->Output = U;
    pV->LastError = pV->Error;
}

void Fuzzy_Pare_init(void) {
    // 速度控制器參數
    fuzzy_spd.KE = 1.0f;    // 誤差增益
    fuzzy_spd.KEC = 1.0f;   // 誤差變化增益
    fuzzy_spd.KU = 2.0f;    // 輸出增益
    fuzzy_spd.Umax = 5950;  // 最大輸出
    fuzzy_spd.Umin = 0;     // 最小輸出
    fuzzy_spd.LastError = 0;
    fuzzy_spd.Error = 0;
    fuzzy_spd.Output = 0;
    
    // 電流控制器參數
    fuzzy_curr.KE = 1.0f;   // 誤差增益
    fuzzy_curr.KEC = 1.0f;  // 誤差變化增益
    fuzzy_curr.KU = 4.0f;   // 輸出增益
    fuzzy_curr.Umax = 5950; // 最大輸出
    fuzzy_curr.Umin = 0;    // 最小輸出
    fuzzy_curr.LastError = 0;
    fuzzy_curr.Error = 0;
    fuzzy_curr.Output = 0;
}

