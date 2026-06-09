/*
 * VORTEX modu - Celebi Takimi ozel ucus modu (sade ve guvenli surum).
 *
 * Davranis:
 *   - Sabit yatis acisiyla surekli daire cizer (yapay ufuk yatik durur)
 *   - Pitch + gazi TECS ile otomatik tutar (irtifa/hizi kendisi yonetir)
 *
 * NOT: Private API'lere dokunmaz, bu yuzden guvenle derlenir.
 *      Irtifa salinimi ileride public bir yontemle eklenebilir.
 *      Once SITL'de test edin; gercek ucusta yedek pilot MANUAL/FBWA/RTL'e
 *      gecmeye hazir olsun, yuksek irtifada deneyin.
 */
#include "mode.h"
#include "Plane.h"

// Sabit yatis acisi (centi-derece): 2500 = 25 derece
static const int32_t VORTEX_BANK_CD = 2500;

bool ModeVortex::_enter()
{
    // Ek bir hazirliga gerek yok; giriste mevcut irtifa/hiz korunur.
    return true;
}

void ModeVortex::update()
{
    // 1) Sabit yatis -> ucak surekli daire cizer (roll limiti icinde guvenli)
    plane.nav_roll_cd = constrain_int32(VORTEX_BANK_CD,
                                        -plane.roll_limit_cd,
                                        plane.roll_limit_cd);

    // 2) Pitch ve gazi TECS ile otomatik hesapla (irtifa/hiz otomatik)
    plane.calc_nav_pitch();
    plane.calc_throttle();
}
