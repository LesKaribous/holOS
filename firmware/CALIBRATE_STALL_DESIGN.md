# Auto-calibration de la détection de collision (`calibrateStall`)

> **Objectif** : rendre la détection de collision par **stagnation** fiable,
> sans faux-positifs pendant les moves normaux ni faux-négatifs contre un mur.
> Inspiré du pattern `probeBorder` : on utilise la bordure comme butée connue
> pour enchaîner des **tests vrai-positif** et **tests faux-positif**, puis on
> règle automatiquement `stall.stag_move_mm` et `stall.stag_time`.

---

## 1. Pourquoi la stagnation ? (et pas les deux autres méthodes)

| Méthode | Fenêtre | Forces | Limitations |
|---|---|---|---|
| **Sliding-window displacement** (legacy) | `DELAY_MS` + `PERIOD_MS` ≈ 1.5 s | Robuste sur mouvement long | Trop lent, pas fiable pour contact basse-vitesse |
| **Velocity mismatch** (cmdVel vs otosVel) | `velStallTimeS` = 0.2 s | Rapide, per-axis (X, Y, rot) | Rate `cmdVel < velCmdMinMmS = 30 mm/s` : **invisible en approche lente** (le cas de `probeBorder` à feedrate 0.3) |
| **Position stagnation** (erreur PID non décroissante) | `stagTimeS` = 0.4 s | Fonctionne à toute vitesse, capte la "butée" même si le robot gratte | Peut glisser sur oscillations (voir §2.3) |

En compétition on a besoin :
- de capter les butées lentes (`probeBorder` à 0.2 feedrate)
- de capter les butées en mouvement normal (feedrate 1.0)

→ **Stagnation est le seul dénominateur commun.** On calibre uniquement ça.
Pendant la calibration on **désactive** velocity mismatch (met `velCmdMinMmS` à une valeur énorme) pour isoler la contribution de la stagnation.

---

## 2. Rappels sur le fonctionnement actuel du stall

### 2.1 Paramètres pertinents (extraits de `stallDetector.h` + loader `positionController.cpp:73-88`)

```cpp
// Stagnation — ce qu'on va calibrer
sc.stagMoveMm   = RuntimeConfig::getFloat("stall.stag_move_mm",  0.5f);  // mm d'erreur à gagner / 100 ms
sc.stagTimeS    = RuntimeConfig::getFloat("stall.stag_time",     0.4f);  // durée de stagnation pour déclencher (s)
sc.stagErrorMm  = RuntimeConfig::getFloat("stall.stag_error_mm", 3.0f);  // seuil sous lequel on ignore (settling)
```

Valeurs par défaut : erreur doit **diminuer de 0.5 mm par 100 ms** pendant **400 ms** pour ne *pas* stagner. Si elle diminue moins, on accumule du temps de stagnation ; quand l'accu dépasse `stagTimeS`, on flag `stalledX`/`stalledY`.

### 2.2 Chemin du flag vers l'annulation du move

```
PID tick (500 Hz)
 └─ m_stall.updateStagnation(pos, target, dt)
       └─ si stagnation → m_stats.stalledX = true
                            m_stats.velTriggered = true
 └─ PositionController::onUpdate()
       └─ if m_stall.isStalledAny() && m_stallEnabled → m_stalledFlag = true
motion.update() (1 kHz)
 └─ if cruise.isStalled() && cancelOnStall → cruise.cancel()
 └─ motion.setAbsPosition(corrected) + cruise.snapAxisTarget + stall.clearStalledX/Y()
```

### 2.3 Subtilité importante

`updateStagnation` compare `|error|` au début et à la fin d'une fenêtre 100 ms.
Si le robot **oscille** autour du mur (petits glissements latéraux), l'erreur peut
varier sans que la position avance réellement. La stagnation actuelle peut rater
ce cas. **Pour la calibration on s'en fiche** : on impacte frontalement un mur,
l'oscillation est négligeable. Mais il faut savoir que pour des scénarios
"grating contact" (frottement latéral) on pourrait vouloir en plus un
"position-delta" stagnation (différent de `errorDelta`). C'est une amélioration
future, hors scope de ce ticket.

---

## 3. Architecture proposée

```
┌─────────────────────────────────────────────────┐
│ command_calib_stall (commands.cpp)              │
│   args : [face, maxIterations]                  │
│   ↓                                             │
│ calibrateStall(RobotCompass face, int nIter)    │  (strategy.cpp)
│   ↓                                             │
│ probeBorder(WEST, face, clearance=500)          │  recalage initial
│   ↓                                             │
│ for i in [0, nIter):                            │
│   ┌─ PHASE A : test vrai-positif              ─┤
│   │   advance 500 mm vers mur                   │
│   │   reverse jusqu'à stall OU timeout 5 s      │
│   │   → si timeout : stagMove ×= 1.4           │ (rendre plus sensible)
│   │     stagTime  *= 0.8                       │
│   │     continue                                │
│   ┌─ PHASE B : test faux-positif              ─┤
│   │   probeBorder(WEST, face, 0) + avance 500  │
│   │   recule 400 mm, stall actif                │
│   │   → si stall déclenché avant 400 mm :      │
│   │     stagMove *= 0.7                        │ (rendre moins sensible)
│   │     stagTime *= 1.2                        │
│   │     continue                                │
│   └─ les deux passent : converged ✓ break      │
│   ↓                                             │
│ persist via RuntimeConfig::setFloat(...)        │
│ report via g_lastCalibReport                    │
└─────────────────────────────────────────────────┘
```

### 3.1 Pourquoi descente adaptative et pas grid-search

- **Monotone** : si `stagMove` est trop élevé → faux-négatifs (timeouts) ; trop bas → faux-positifs. La recherche est quasi-1D.
- **Rapide** : 3-5 itérations suffisent typiquement (vs 25 pour une 5×5 grille).
- **Safe** : on converge vers la frontière, pas de valeurs extrêmes testées longtemps.

### 3.2 Pourquoi on fige `stagErrorMm`

`stagErrorMm` filtre le bruit de settling (sous 3 mm d'erreur, on ignore). Le
régler plus bas provoque des faux-positifs en fin de move. Le régler plus haut
masque les contacts légers. La valeur par défaut 3 mm est éprouvée — on n'y
touche pas dans cette calibration.

---

## 4. Implémentation

### 4.1 Déclaration dans `strategy.h`

```cpp
// Auto-calibration de la détection de collision par stagnation.
// Enchaîne tests vrai-positif (bump mur) et faux-positif (course libre 400 mm)
// pour régler stall.stag_move_mm / stall.stag_time jusqu'à convergence.
//
//  face        : face du robot utilisée pour heurter la bordure WEST (x=0).
//                Typiquement RobotCompass::AB.
//  maxIter     : nombre max d'itérations de l'ajustement (default 6).
// Renvoie via g_lastCalibReport :
//   "kind=calib_stall iter=<n> stag_move=<mm> stag_time=<s> status=<converged|timeout|abandoned>"
FLASHMEM void calibrateStall(RobotCompass face = RobotCompass::AB, int maxIter = 6);
```

### 4.2 Implémentation dans `strategy.cpp` (à insérer après `probeBorder`, ligne ~524)

```cpp
// ============================================================
// Auto-calibration de la détection de collision (stagnation)
// ============================================================
//
// Pattern : probeBorder → avance 500 mm (sûr) → reverse vers mur.
// On cherche le (stag_move_mm, stag_time) qui déclenche en contact et
// pas en course libre.
//
// IMPORTANT : on désactive temporairement le velocity-mismatch en poussant
// velCmdMinMmS très haut pour isoler la contribution de la stagnation.
// On remet les valeurs à la fin.
// ============================================================
FLASHMEM void calibrateStall(RobotCompass face, int maxIter) {
    const float APPROACH_FREE = 500.0f;   // mm avance en zone libre
    const float PROBE_REVERSE = 400.0f;   // mm de course libre pour test faux-positif
    const float PROBE_BUMP    = 600.0f;   // mm max de course pour bumper le mur
    const float FEEDRATE      = 0.4f;     // vitesse dégradée pour calibration
    const uint32_t TIMEOUT_MS = 5000;     // 5 s max par probe

    boolean wasAbsolute = motion.isAbsolute();
    float   savedFeed   = motion.getFeedrate();

    // Snapshot des paramètres courants pour rollback éventuel
    const float saved_stagMove  = RuntimeConfig::getFloat("stall.stag_move_mm", 0.5f);
    const float saved_stagTime  = RuntimeConfig::getFloat("stall.stag_time",    0.4f);
    const float saved_velCmdMin = RuntimeConfig::getFloat("stall.vel_cmd_min", 30.0f);

    // Valeurs de travail
    float stagMove = saved_stagMove;
    float stagTime = saved_stagTime;

    // Isole la stagnation : on rend velocity-mismatch aveugle
    RuntimeConfig::setFloat("stall.vel_cmd_min", 1.0e6f);

    motion.setFeedrate(FEEDRATE);

    bool converged  = false;
    bool abandoned  = false;
    int  iter       = 0;

    // Helper : les probeBorder intermédiaires ne doivent PAS utiliser les
    // params de test (risque de false-positive pendant la recalage elle-même).
    // On push des valeurs safe par défaut pendant le probeBorder, puis on
    // restaure les valeurs de test.
    auto safeProbeBorder = [&](TableCompass tc, RobotCompass rc, float clearance) {
        RuntimeConfig::setFloat("stall.stag_move_mm", saved_stagMove);
        RuntimeConfig::setFloat("stall.stag_time",    saved_stagTime);
        RuntimeConfig::setFloat("stall.vel_cmd_min",  saved_velCmdMin);  // réactive velocity pour probe
        probeBorder(tc, rc, clearance);
        // Remets velocity-mismatch aveugle (reste aveugle pour le reste du test)
        RuntimeConfig::setFloat("stall.vel_cmd_min", 1.0e6f);
    };

    // Recalage initial contre la bordure WEST (avec params safe)
    Console::info("CalibStall") << "Initial recalage on WEST border" << Console::endl;
    safeProbeBorder(TableCompass::WEST, face, /*clearance*/ APPROACH_FREE);

    for (iter = 0; iter < maxIter; iter++) {
        Console::info("CalibStall")
            << "Iter " << iter
            << " stagMove=" << stagMove
            << " stagTime=" << stagTime
            << Console::endl;

        // Push les paramètres courants dans RuntimeConfig (relu au prochain start())
        RuntimeConfig::setFloat("stall.stag_move_mm", stagMove);
        RuntimeConfig::setFloat("stall.stag_time",    stagTime);

        // ─────────────────────────────────────────────────────────
        // PHASE A : test VRAI-POSITIF (doit stall contre le mur)
        // ─────────────────────────────────────────────────────────
        motion.setRelative();
        motion.disableCruiseMode();
        motion.cancelOnStall(true);

        // Avance vers le mur : signe + car getCompassOrientation(face) pointe
        // DÉJÀ vers le mur (face alignée par probeBorder). Convention identique
        // à probeBorder interne : +D = vers le mur, -D = éloigne du mur.
        uint32_t t0 = millis();
        async motion.goPolar(getCompassOrientation(face), +PROBE_BUMP);
        uint32_t elapsedA = millis() - t0;

        motion.cancelOnStall(false);
        motion.enableCruiseMode();

        bool stalledA = (elapsedA < TIMEOUT_MS);  // cancel = stall détecté OU mur atteint
        // Note : si mur atteint sans stall, motion finit le move normalement et
        // elapsedA reflète le temps réel. On considère stall OK ssi on a été stoppé
        // avant d'avoir parcouru PROBE_BUMP. On peut affiner avec la position OTOS.

        Vec3 posAfterA = localisation.getPosition();
        Console::info("CalibStall")
            << " A: elapsed=" << elapsedA
            << " ms, posX=" << posAfterA.x
            << (stalledA ? " → STALL" : " → NO STALL")
            << Console::endl;

        if (!stalledA) {
            // Rien n'a stoppé le robot ou le move a atteint 5 s → stagnation trop rigide.
            // On rend plus sensible (accepte moins de progrès avant de flag).
            stagMove *= 1.4f;
            stagTime *= 0.8f;
            // Clamp anti-divergence
            stagMove = std::min(stagMove, 4.0f);
            stagTime = std::max(stagTime, 0.10f);
            Console::warn("CalibStall") << "Phase A failed — loosening params" << Console::endl;
            // Re-recaler pour être sûr de la position avant prochain essai
            safeProbeBorder(TableCompass::WEST, face, APPROACH_FREE);
            continue;
        }

        // Stall vrai-positif OK. On re-recale proprement (params safe).
        safeProbeBorder(TableCompass::WEST, face, APPROACH_FREE);

        // ─────────────────────────────────────────────────────────
        // PHASE B : test FAUX-POSITIF (ne doit PAS stall en course libre)
        // ─────────────────────────────────────────────────────────
        motion.setRelative();
        motion.disableCruiseMode();
        motion.cancelOnStall(true);

        // Éloigne-toi du mur : signe - (convention inverse). 400 mm en zone
        // libre (après probeBorder on est à ~clearance=500 mm du mur).
        uint32_t t1 = millis();
        async motion.goPolar(getCompassOrientation(face), -PROBE_REVERSE);
        uint32_t elapsedB = millis() - t1;

        motion.cancelOnStall(false);
        motion.enableCruiseMode();

        // Si le move a été annulé → faux-positif. Détection via stats OTOS :
        // distance parcourue réelle ~= PROBE_REVERSE si OK, sinon moins.
        Vec3 posAfterB = localisation.getPosition();
        // Le robot est parti de ~500 mm après recalage, doit avoir couvert ~400 mm.
        // On tolère 10% d'erreur métrique.
        float coveredB = fabsf(posAfterB.x - APPROACH_FREE);
        bool  falsePositive = (coveredB < 0.85f * PROBE_REVERSE);

        Console::info("CalibStall")
            << " B: elapsed=" << elapsedB
            << " ms, covered=" << coveredB
            << (falsePositive ? " → FALSE POSITIVE" : " → OK")
            << Console::endl;

        if (falsePositive) {
            // Trop sensible → relâcher.
            stagMove *= 0.7f;
            stagTime *= 1.2f;
            stagMove = std::max(stagMove, 0.1f);
            stagTime = std::min(stagTime, 1.0f);
            Console::warn("CalibStall") << "Phase B failed — tightening params" << Console::endl;
            probeBorder(TableCompass::WEST, face, APPROACH_FREE);
            continue;
        }

        // Les deux phases passent.
        converged = true;
        break;
    }

    if (!converged) {
        abandoned = true;
        Console::error("CalibStall")
            << "Did not converge in " << maxIter << " iterations. "
            << "Rolling back to previous params." << Console::endl;
        // Rollback : on remet les paramètres précédents
        stagMove = saved_stagMove;
        stagTime = saved_stagTime;
    }

    // Écrit les valeurs finales (convergées ou rollback) dans RuntimeConfig.
    // Le prochain move les relira automatiquement dans start().
    RuntimeConfig::setFloat("stall.stag_move_mm", stagMove);
    RuntimeConfig::setFloat("stall.stag_time",    stagTime);
    // Restaure velocity-mismatch
    RuntimeConfig::setFloat("stall.vel_cmd_min",  saved_velCmdMin);

    // Report pour le Jetson
    snprintf(g_lastCalibReport, sizeof(g_lastCalibReport),
             "kind=calib_stall iter=%d stag_move=%.3f stag_time=%.3f status=%s",
             iter, stagMove, stagTime,
             converged ? "converged" : (abandoned ? "abandoned" : "timeout"));
    Console::info("CalibStall") << g_lastCalibReport << Console::endl;

    // Restauration état motion
    if (wasAbsolute) motion.setAbsolute();
    motion.setFeedrate(savedFeed);
}
```

### 4.3 Commande dans `commands.cpp` (à insérer après `command_probe_open`, ligne ~400)

```cpp
/**
 * calib_stall(face[, maxIter]) — auto-tune de la détection de collision.
 *
 * Args:
 *   face    : face du robot utilisée pour frapper le mur WEST
 *             (A | AB | B | BC | C | CA). Défaut AB.
 *   maxIter : nb max d'itérations d'ajustement. Défaut 6.
 *
 * Procédure :
 *  1. Recalage initial contre la bordure WEST (x=0)
 *  2. Test vrai-positif : recule vers le mur, doit stall
 *     - échec → assouplit stag_move × 1.4, stag_time × 0.8
 *  3. Test faux-positif : avance 400 mm libre, ne doit PAS stall
 *     - échec → durcit stag_move × 0.7, stag_time × 1.2
 *  4. Persiste via RuntimeConfig (stall.stag_move_mm, stall.stag_time)
 *
 * Report : kind=calib_stall iter=<n> stag_move=<mm> stag_time=<s> status=<...>
 */
FLASHMEM void command_calib_stall(const args_t& args) {
    RobotCompass rc = RobotCompass::AB;
    int maxIter     = 6;

    if (args.size() >= 1) {
        String faceStr = args[0];
        if      (faceStr.equalsIgnoreCase("A"))   rc = RobotCompass::A;
        else if (faceStr.equalsIgnoreCase("AB"))  rc = RobotCompass::AB;
        else if (faceStr.equalsIgnoreCase("B"))   rc = RobotCompass::B;
        else if (faceStr.equalsIgnoreCase("BC"))  rc = RobotCompass::BC;
        else if (faceStr.equalsIgnoreCase("C"))   rc = RobotCompass::C;
        else if (faceStr.equalsIgnoreCase("CA"))  rc = RobotCompass::CA;
    }
    if (args.size() >= 2) {
        maxIter = args[1].toInt();
        if (maxIter < 1)  maxIter = 1;
        if (maxIter > 20) maxIter = 20;
    }

    calibrateStall(rc, maxIter);
    // g_lastCalibReport est rempli par calibrateStall()
}
```

### 4.4 Enregistrement dans le dispatch des commandes

Dans la table où `probe_open` est enregistré (`commands.cpp`, cherche `"probe_open"`) :

```cpp
{"calib_stall",  command_calib_stall,  false},
```

(Adapte le `false` selon le 3ème champ attendu par ta structure — c'est le même que `probe_open`.)

---

## 5. Points critiques / pièges

### 5.1 L'annulation par timeout vs par stall

Le `async motion.goPolar(...)` rend la main quand :
- le move a complété normalement,
- OU le cruise a été cancel (stall détecté),
- OU le timeout motion (hardcoded 5 s) a cancel.

Dans mon code je détecte le "stall détecté" via `elapsed < TIMEOUT_MS`. Ce n'est
pas parfait : si le timeout motion est à **5 000 ms** et mon seuil aussi, il y a
ambiguïté. **Remède** :
- mettre le seuil à `TIMEOUT_MS - 200` pour garder une marge, OU
- lire `motion.wasStalled()` / `cruise.isStalled()` juste après l'async (si cette
  API existe — à vérifier dans `motion.h`). Si elle n'existe pas, c'est une
  petite amélioration à ajouter :
  ```cpp
  bool Motion::lastMoveStalled() const { return cruise_controller.isStalled(); }
  ```

### 5.2 Feedrate de calibration

Je mets `FEEDRATE = 0.4f` comme compromis. Trop lent → velocity-mismatch serait
aveugle (mais c'est voulu ici, je l'ai désactivé). Trop rapide → rebonds sur le
mur, bruit. 0.4 correspond à ~200 mm/s pour un robot typique → 500 mm en 2.5 s,
ce qui laisse de la marge dans le timeout 5 s.

### 5.3 Pourquoi réafficher `probeBorder` entre chaque itération

Sinon la position estimée dérive (contact mur → correction → mouvement → erreur
cumulée OTOS). Un nouveau `probeBorder` réinitialise le cadre de référence à
chaque passe. Coût : ~2-3 s par itération. Acceptable.

### 5.4 Clamp anti-divergence

```cpp
stagMove ∈ [0.1, 4.0] mm / 100 ms
stagTime ∈ [0.10, 1.0] s
```
Valeurs hors de ces plages sont soit absurdes (0.01 mm = bruit capteur → stall
permanent) soit inefficaces (4 s = robot coincé 4 s avant de s'arrêter).

### 5.5 Facteurs 1.4 / 0.8 / 0.7 / 1.2

Non symétriques exprès. On préfère **pêcher par excès de prudence** côté
faux-positif (facteur 0.7, plus agressif) que côté faux-négatif. Un robot qui
s'arrête trop tôt est récupérable ; un robot qui bulldoze une pièce ne l'est
pas.

### 5.6 Ce qui n'est PAS dans le scope

- Calibration de `stagErrorMm` (figé à 3 mm, changer demande refactor).
- Calibration par axe Y séparé (on calibre contre WEST = axe X). On pourrait
  doubler avec un `calibrateStall(rc, NORTH)` plus tard.
- Calibration de la rotation (`velRotMin`/`velRotMax`).

---

## 6. Procédure utilisateur

```
$ probe_open WEST AB             # sanity-check : le robot doit buter et stall
$ calib_stall AB 6               # lance la calibration (6 itérations max)
... ~30-60 s ...
$ probe_open WEST AB             # re-test : doit stall proprement
$ go x 500                       # re-test : doit rouler 500 mm sans false positive
```

Si `status=converged` dans le report : OK, les valeurs sont en vigueur jusqu'au
prochain reboot (`RuntimeConfig` est volatile). Pour persister :
- soit push les valeurs trouvées dans `Settings::Motion::Stall::*` (hardcoded),
- soit ajouter un `RuntimeConfig::saveToEEPROM()` si la Teensy en a un dispo.

## 7. Checklist d'intégration

- [ ] Déclarer `calibrateStall()` dans `strategy.h`
- [ ] Implémenter le corps dans `strategy.cpp` après `probeBorder` (§4.2)
- [ ] `#include <algorithm>` en tête de `strategy.cpp` (pour `std::min`/`std::max`)
- [ ] `#include "config/runtime_config.h"` si absent
- [ ] Ajouter `command_calib_stall` dans `commands.cpp` (§4.3)
- [ ] Enregistrer dans la table dispatch des commandes (§4.4)
- [ ] Tester `calib_stall AB` en mode manuel
- [ ] Vérifier le report via `calib_report` (si la commande existe) ou `serial`

## 8. Améliorations futures (hors scope)

1. **Stall par delta-position au lieu de delta-erreur** : ajouterait un 4e mode
   au `StallDetector` pour capter les frottements latéraux (robot qui gratte le
   mur au lieu de buter frontalement).
2. **Calibration multi-axes** : enchaîner WEST + NORTH pour couvrir X et Y.
3. **Persistance EEPROM** : sauvegarder dans `Calibration::Current` (+ load
   au boot) plutôt que `RuntimeConfig` volatile.
4. **Auto-déclenchement au démarrage** : si les valeurs sont "usine", lancer
   `calibrateStall` dans `recalage()` une seule fois.
