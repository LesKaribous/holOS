
#include "utils/geometry.h"

void recalage();          // Classical: homography lock + prise d'origine
void visionRecalage();    // Multi-pose parallax sweep (requires recalage() first)


void calibrate();
void probeBorder(TableCompass tc, RobotCompass rc, float clearance, float approachDist = 200.0, float probeDist = 80.0, float feedrate = 0.2 );

// ─────────────────────────────────────────────────────────────────────────────
//  calibrateStall — Auto-tune de la détection de collision par stagnation.
//
//  Enchaîne tests vrai-positif (bump mur, doit stall) et faux-positif (course
//  libre, ne doit pas stall) contre la bordure WEST, en ajustant les params
//  RuntimeConfig  stall.stag_move_mm  et  stall.stag_time  jusqu'à convergence.
//
//  Les autres méthodes (velocity mismatch) sont temporairement désactivées
//  pour isoler la stagnation.
//
//  Args :
//    face    : face robot utilisée pour heurter la bordure WEST (défaut AB)
//    maxIter : itérations max avant abandon (défaut 6)
//
//  Retourne le résultat pour que la commande wrapper puisse le pousser en
//  télémétrie (T:cal frame).
// ─────────────────────────────────────────────────────────────────────────────
struct StallCalibResult {
    bool  converged  = false;  // les deux tests passent avec les params retenus
    int   iter       = 0;      // nombre d'itérations effectuées
    float stagMoveMm = 0.0f;   // valeur finale retenue (converged ou rollback)
    float stagTimeS  = 0.0f;
    bool  phaseAOk   = false;  // dernier test vrai-positif : stall déclenché
    bool  phaseBOk   = false;  // dernier test faux-positif : pas de stall en zone libre
};
StallCalibResult calibrateStall(RobotCompass face = RobotCompass::AB, int maxIter = 6);