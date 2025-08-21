# drone-core

Bu depo, bir dronun kontrol edilmesine yönelik temel yazılım bileşenlerini içerir. Görev mantığı, uçuş kontrolcüleri ve çeşitli yardımcı modülleri kapsar.

## Proje Yapısı

Proje aşağıdaki klasör yapısına sahiptir:

* **controllers**: Drone için farklı kontrolcü modüllerini içerir.
  * `xbee_controller.py`: XBee modülü üzerinden iletişimi yönetir.
  * `mavsdk_controller.py`: MAVSDK aracılığıyla drone ile haberleşmeyi sağlar.
  * `drone_controller.py`: Sık kullanılan dron işlemlerini adım kontrol mekanizmasına uygun şekilde barındırır.
  * `offboard_controller.py`: PID ve APF işlemlerini içerir ve Offboard modu ile alakalı işlemleri barındırır.
  * `step_controller.py`: Adım tabanlı hareketleri ve dronlar arası senkronizasyonu yöneten kontrol mekanizması.

* **core**: Temel bileşenleri barındırır.
  * `drone.py`: Dronun tanımlandığı ana modüldür, diğer modüller bu modül aracılığıyla drona erişebilir, dronun max hızı, PID değerleri vb. buradan ayarlanır.
  * `mission.py`: Görevler için temel sınıfları ve mantığı tanımlar, tüm görevler bu sınıfın birer türetilmiş versiyonudur.

* **missions**: Görevleri barındırır.

* **utils**: Sistemin farklı bölümleri tarafından kullanılan yardımcı modüller.
  * `apf.py`: Yapay Potansiyel Alan yöntemiyle çarpışma önleme algoritması.
  * `pid.py`: PID ile hedef noktaya ilerleme algoritması.
  * `socket_communication.py`: İletişim bağlantılarını socket ile sanal olarak simüle eder.
  * `formation_utilities.py`: Formasyon vb. işlemler için gerekli ortak fonksiyonları (lat_lon -> Metre vb.) barındırır.

* **Başlatma Scriptleri**
  * `service.py`: Arayüz ile beraber görevleri çalıştıran servis scripti, aynı zamanda XBee'leri tek bir bilgisayara bağlayarak simülasyonda arayüzün kullanılmasına olanak sağlar.
  * `tester.py`: Önceden tanımlı parametreler ile görevlerin test amaçlı çalıştırılmasını sağlar.

## Başlarken

Simülasyonun çalışabilmesi için öncelikle PX4-Autopilot’un kurulu olması gerekir.

### PX4 Kurulumu
1. PX4 Reposunu klonlayın
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
2. Otomatik Kurulum Dosyasını Çalıştırın
```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

### Tek Dron ile Simülasyon

Tek Dron ile simülasyon ortamı oluşturmak için `launch_drones.sh` scriptini aşağıdaki örnekteki gibi kullanın

```bash
# Örneğin:
# ./launch_drones.sh <dron_sayısı>
./launch_drones.sh 1
```

### Birden Fazla Dron ile Simülasyon
Tıpkı tek dron simülasyonundaki gibi `launch_drones.sh` scriptini kullanıp, dron sayısını istediğiniz şekilde girin
```bash
# Örneğin:
# ./launch_drones.sh <dron_sayısı>
./launch_drones.sh 3
```
Ekranınızda Gazebo gözükecektir, bağlantı portlarını ve kodlarını görüntülemek için scripti çalıştırdığınız terminal penceresine bakın, her bir dron için örnek MAVSDK bağlantı kodunu göreceksiniz.

Uçuş kanıt videosunun görev kodlarındaki gibi bir `sim_instance` oluşturup dron seçimini hızlandırmanızı öneririm.

PX4 Log kayıtları `~/drone_logs` isimli klasörde bulunur

**Desteklenen Gazebo Sürümleri:**
- Classic
- Harmonic

### Arayüz ile Simülasyon Yapma
```bash
python service.py --drone_id <simulasyondaki_dron_id_degeri> --is_sim <simulasyon_mu>

# Örneğin simülasyondaki 1. dron için
python service.py --drone_id 0 --is_sim 1
# 2. Dron için
python service.py --drone_id 1 --is_sim 1
```
* `drone_id`: değişkeni 0'dan başlayarak eklediğiniz dron sayısının bir eksiğine kadar ilerler.
* `is_sim`: 1 veya 0 değeri alır, 1 evet; 2 hayıra karşılık gelir, çalışma ortamının simülasyon olup olmadığını sorar, verilen değere göre drona bağlanırken kullanılan portu ayarlar.

Her bir dron için yeni bir terminal açılır ve dronun id değeri ve çalışma ortamı girilerek o drona bağlanılır, dronlar arayüze listelendiğinde ise görev parametreleri arayüzden ayarlanarak göreve başlangıç komutu verilir.

### Test Çalıştırma

Herhangi bir kontrolcü ya da görev modülünü çalıştırmak için projenin kök dizininden `tester.py` scriptini kullanabilirsiniz.

Script, çalıştırılacak modülün Python yolu şeklinde bir argüman alır.

**Kullanım biçimi:**

```bash
python tester.py modul.konumu calistirilacak_dron(sim_instance)
```

**Örneğin:**

`./missions/ucus_kanit.py` yolundaki görevi 1. dron ile çalıştırmak için:

```bash
python tester.py missions.formasyon 0
```
2 ve 3. dronlar için ise 0 sayısını 1,2... şeklinde yükseltebilirsiniz

## Bilinen Buglar ve Sorunlar

- [x] Formasyona geçişte irtifa düşüşü (%90 Düzeltildi)
- [x] Formasyona geçişte diğer dronlara çok yakın olan tehlikeli rotalar izleme (%90 Düzeltildi)
- [x] Çok yakın mesafelerde zaman zaman PID kitlenmesi
- [x] Sıklığı azaltılmış olsa da irtifa kontrolünde zaman zaman yaşanan kitlenme problemi (Dronun doğru irtifaya çıkamamasıyla veya irtifa hesabının hatalı olmasına bağlı olabilir)
- [x] Komşu dronların ID hesaplarının XBee modunda sorunlu olması
- [ ] Birey ekle/çıkar görevinde iniş yapan dronun geri kalkamaması
- [ ] Birey ekle/çıkar görevinde bazen 2 dronun da formasyondan çıkma emri alması

## Tamamlanan Görevler

- 3B Formasyon Görevi
  - [x] Simülasyon Testleri
  - [x] Saha Testleri
- Formasyon ile Navigasyon Görevi
  - [x] Simülasyon Testleri
  - [x] Saha Testleri
- Birey Ekleme Çıkartma Görevi
  - [x] Simülasyon Testleri
  - [x] Saha Testleri (%90)
- Sürü Keşif Görevi
  - [x] Simülasyon Testleri
  - [x] Saha Testleri