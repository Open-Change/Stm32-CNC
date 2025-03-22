/********************************************************************
 * Açıklama: CoreXY Kinematiği Uygulaması
 * 
 * Bu dosya, 3D yazıcılarda yaygın olarak kullanılan CoreXY mekanizmasının kinematiğini uygular.
 * CoreXY mekanizması, X ve Y eksenlerinin hareketini koordineli bir şekilde kontrol etmek için iki motor kullanır.
 * Bu uygulama, basit bir kinematik örneği olan trivkins.c dosyasından uyarlanmıştır.
 * 
 * Referans: http://corexy.com/theory.html
 * 
 * kinematicsForward fonksiyonu, eklem pozisyonlarına dayanarak Kartezyen koordinatları (X, Y, Z, vb.) hesaplar.
 * kinematicsInverse fonksiyonu, Kartezyen koordinatlarına dayanarak eklem pozisyonlarını hesaplar.
 * 
 * HAL (Donanım Soyutlama Katmanı), yazılımın motorları kontrol etmesini sağlamak için donanım ile arayüz oluşturur.
 ********************************************************************/

#include "motion.h"       // Hareket kontrol tanımlamaları
#include "hal.h"         // Donanım Soyutlama Katmanı (HAL) tanımlamaları
#include "rtapi.h"       // Gerçek Zamanlı API tanımlamaları
#include "rtapi_app.h"   // RTAPI uygulama tanımlamaları
#include "rtapi_math.h"  // RTAPI için matematik fonksiyonları
#include "rtapi_string.h"// RTAPI için string fonksiyonları
#include "kinematics.h"  // Kinematik tanımlamaları

// Veri yapısı: Motor hareketlerini saklamak için kullanılır
static struct data {
    hal_s32_t joints[EMCMOT_MAX_JOINTS]; // Eklem pozisyonları
    hal_float_t motor1; // Birinci motorun hareket değeri
    hal_float_t motor2; // İkinci motorun hareket değeri
} *data;

// Motor hareket değerlerine erişim için kısayollar
#define Motor1 (data->motor1) // Birinci motorun hareket değeri
#define Motor2 (data->motor2) // İkinci motorun hareket değeri

/********************************************************************
 * kinematicsForward: İleri Kinematik Hesaplama
 * 
 * Bu fonksiyon, eklem pozisyonlarını (joints) alır ve Kartezyen koordinatlarını (pos) hesaplar.
 * CoreXY mekanizması için:
 * - X pozisyonu, iki motorun hareketlerinin ortalamasıdır.
 * - Y pozisyonu, iki motorun hareketlerinin farkının yarısıdır.
 * - Diğer eksenler (Z, A, B, C, U, V, W) doğrudan eklem pozisyonlarına eşittir.
 ********************************************************************/
int kinematicsForward(const double *joints, EmcPose *pos,
                     const KINEMATICS_FORWARD_FLAGS *fflags,
                     KINEMATICS_INVERSE_FLAGS *iflags) {
    (void)fflags; // Kullanılmayan parametreler
    (void)iflags; // Kullanılmayan parametreler

    // CoreXY kinematiği hesaplamaları
    pos->tran.x = 0.5 * (joints[0] + joints[1]); // X = (Motor1 + Motor2) / 2
    pos->tran.y = 0.5 * (joints[0] - joints[1]); // Y = (Motor1 - Motor2) / 2
    pos->tran.z = joints[2]; // Z doğrudan eklem pozisyonuna eşit
    pos->a = joints[3];      // A doğrudan eklem pozisyonuna eşit
    pos->b = joints[4];      // B doğrudan eklem pozisyonuna eşit
    pos->c = joints[5];      // C doğrudan eklem pozisyonuna eşit
    pos->u = joints[6];      // U doğrudan eklem pozisyonuna eşit
    pos->v = joints[7];      // V doğrudan eklem pozisyonuna eşit
    pos->w = joints[8];      // W doğrudan eklem pozisyonuna eşit

    return 0; // Başarılı
}

/********************************************************************
 * kinematicsInverse: Ters Kinematik Hesaplama
 * 
 * Bu fonksiyon, Kartezyen koordinatlarını (pos) alır ve eklem pozisyonlarını (joints) hesaplar.
 * CoreXY mekanizması için:
 * - Motor1 = X + Y
 * - Motor2 = X - Y
 * - Diğer eksenler (Z, A, B, C, U, V, W) doğrudan Kartezyen koordinatlarına eşittir.
 ********************************************************************/
int kinematicsInverse(const EmcPose *pos, double *joints,
                     const KINEMATICS_INVERSE_FLAGS *iflags,
                     KINEMATICS_FORWARD_FLAGS *fflags) {
    (void)iflags; // Kullanılmayan parametreler
    (void)fflags; // Kullanılmayan parametreler

    // CoreXY ters kinematiği hesaplamaları
    joints[0] = pos->tran.x + pos->tran.y; // Motor1 = X + Y
    joints[1] = pos->tran.x - pos->tran.y; // Motor2 = X - Y
    joints[2] = pos->tran.z; // Z doğrudan Kartezyen pozisyonuna eşit
    joints[3] = pos->a;      // A doğrudan Kartezyen pozisyonuna eşit
    joints[4] = pos->b;      // B doğrudan Kartezyen pozisyonuna eşit
    joints[5] = pos->c;      // C doğrudan Kartezyen pozisyonuna eşit
    joints[6] = pos->u;      // U doğrudan Kartezyen pozisyonuna eşit
    joints[7] = pos->v;      // V doğrudan Kartezyen pozisyonuna eşit
    joints[8] = pos->w;      // W doğrudan Kartezyen pozisyonuna eşit

    return 0; // Başarılı
}

/********************************************************************
 * kinematicsHome: Başlangıç Pozisyonu Hesaplama
 * 
 * Bu fonksiyon, başlangıç pozisyonunu (home) hesaplar ve kinematicsForward fonksiyonunu çağırır.
 ********************************************************************/
int kinematicsHome(EmcPose *world, double *joint,
                  KINEMATICS_FORWARD_FLAGS *fflags,
                  KINEMATICS_INVERSE_FLAGS *iflags) {
    *fflags = 0; // Bayrakları sıfırla
    *iflags = 0; // Bayrakları sıfırla
    return kinematicsForward(joint, world, fflags, iflags); // İleri kinematik hesapla
}

/********************************************************************
 * kinematicsType: Kinematik Türünü Belirleme
 * 
 * Bu fonksiyon, kinematik türünü belirler. Bu durumda, hem ileri hem de ters kinematik desteklenir.
 ********************************************************************/
KINEMATICS_TYPE kinematicsType() {
    return KINEMATICS_BOTH; // Hem ileri hem de ters kinematik desteklenir
}

// Kinematik fonksiyonlarını dışa aktar
KINS_NOT_SWITCHABLE
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL"); // Lisans bilgisi

// HAL bileşen kimliği
static int comp_id;

/********************************************************************
 * rtapi_app_main: Uygulama Başlatma Fonksiyonu
 * 
 * Bu fonksiyon, HAL bileşenini başlatır ve motor hareket değerlerini başlangıç durumuna getirir.
 ********************************************************************/
int rtapi_app_main(void) {
    int res = 0;
    comp_id = hal_init("3dkinamtil"); // HAL bileşenini başlat
    if (comp_id < 0) return comp_id; // Hata durumunda çık

    data = hal_malloc(sizeof(struct data)); // Veri yapısı için bellek ayır

    // Motor hareket değerlerini başlangıç durumuna getir
    Motor1 = 0.0;
    Motor2 = 0.0;

    // HAL parametrelerini oluştur
    if ((res = hal_param_float_new("3dkinamtil.Motor1", HAL_RW, &data->motor1, comp_id)) < 0) goto error;
    if ((res = hal_param_float_new("3dkinamtil.Motor2", HAL_RW, &data->motor2, comp_id)) < 0) goto error;

    hal_ready(comp_id); // HAL bileşenini hazır hale getir
    return 0; // Başarılı

error:
    // Hata durumunda HAL bileşenini kapat
    hal_exit(comp_id);
    return res;
}

/********************************************************************
 * rtapi_app_exit: Uygulama Çıkış Fonksiyonu
 * 
 * Bu fonksiyon, HAL bileşenini kapatır.
 ********************************************************************/
void rtapi_app_exit(void) {
    hal_exit(comp_id); // HAL bileşenini kapat
}
