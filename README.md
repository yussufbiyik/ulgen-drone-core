# drone-core

Bu depo, bir dronun kontrol edilmesine yönelik temel yazılım bileşenlerini içerir. Görev mantığı, uçuş kontrolcüleri ve çeşitli yardımcı modülleri kapsar.

## Proje Yapısı

Proje aşağıdaki klasör yapısına sahiptir:

* **controllers**: Drone için farklı kontrolcü modüllerini içerir.

  * `drone_controller.py`: Dronun ana kontrolcüsüdür.
  * `mavsdk_controller.py`: MAVSDK aracılığıyla drone ile haberleşmeyi sağlar.
  * `step_controller.py`: Adım tabanlı hareketleri yöneten kontrolcü.
  * `xbee_controller.py`: XBee modülü üzerinden iletişimi yönetir.
* **core**: Drone yazılımının temel bileşenlerini barındırır.

  * `mission.py`: Görevler için temel sınıfları ve mantığı tanımlar.
* **missions**: Belirli görev uygulamalarını içerir.

  * `son_video.py`: Örnek bir görev senaryosu.
* **utils**: Sistemin farklı bölümleri tarafından kullanılan yardımcı modüller.

  * `apf.py`: Yapay Potansiyel Alan yöntemiyle engel kaçınma algoritması.
  * `pid.py`: PID kontrol algoritması.
  * `safety.py`: Güvenlik kontrolleri ve prosedürleri.
  * `virtual_communication.py`: İletişim bağlantılarını sanal olarak simüle eder.
* **logs**: Sistem tarafından oluşturulan log (günlük) dosyaları burada tutulur.
* `tester.py`: `controllers` veya `missions` klasörlerindeki herhangi bir modülün `main()` fonksiyonunu çalıştırmak için kullanılan betik.
* `requirements.txt`: Projede ihtiyaç duyulan Python bağımlılıklarını listeler.

## Başlarken

### Simülasyon Ortamı

Simülasyonun çalışabilmesi için öncelikle PX4-Autopilot’un kurulu olması gerekir. Kurulumdan sonra simülasyonu şu komutlarla başlatabilirsiniz:

```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```

### Modül Çalıştırma

Herhangi bir kontrolcü ya da görev modülünü çalıştırmak için projenin kök dizininden `tester.py` betiğini kullanabilirsiniz.

Betik, çalıştırılacak modülün Python yolu şeklinde bir argüman alır.

**Kullanım biçimi:**

```bash
python tester.py path.to.module
```

**Örnekler:**

`son_video` görevini çalıştırmak için:

```bash
python tester.py missions.son_video
```

`drone_controller` modülünü çalıştırmak için:

```bash
python tester.py controllers.drone_controller
```

## Test Çalıştırma

Testleri çalıştırmak için yine `tester.py` betiğini kullanabilirsiniz. Örneğin, `tests.test_mission` adlı bir test modülünüz varsa aşağıdaki komutla çalıştırabilirsiniz:

```bash
python tester.py tests.test_mission
```