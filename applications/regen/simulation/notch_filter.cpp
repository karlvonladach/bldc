// Globális vagy statikus változók a szűrő állapotának megőrzéséhez
float A = 0.0f; // Szinuszos komponens súlya
float B = 0.0f; // Koszinuszos komponens súlya

// Tanulási tényező (hangolási paraméter)
// Értéke tipikusan 0.01 és 0.1 között van.
// Nagyobb érték = gyorsabb alkalmazkodás, de több zaj marad.
// Kisebb érték = tökéletesebb simítás, de lassabban tanulja meg a lüktetést.
const float mu = 0.05f; 

/**
 * @brief Adaptív Notch Szűrő futtatása minden mintavételnél
 * @param torque_raw A szenzorból olvasott nyers nyomaték érték
 * @param theta_rad A pedál aktuális szöghelyzete radiánban (0 és 2*PI között)
 * @return float A szűrt, lüktetésmentes nyomaték
 */
float filter_torque_lms(float torque_raw, float theta_rad) {
    // 1. Bázisok számítása a kétszeres szöghelyzetre
    float x1 = sinf(theta_rad);
    float x2 = cosf(theta_rad);
    
    // 2. A lüktetés becslése az aktuális paraméterekkel
    float y_estimated = (A * x1) + (B * x2);
    
    // 3. A tiszta jel kiszámítása (nyers - becsült lüktetés)
    // Ez a hibaérték az LMS algoritmus számára
    float torque_filtered = torque_raw - y_estimated;
    
    // 4. LMS paraméterek frissítése a következő mintához
    A = A + (mu * torque_filtered * x1);
    B = B + (mu * torque_filtered * x2);
    
    return torque_filtered;
}