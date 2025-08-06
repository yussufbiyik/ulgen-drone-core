# drone-core

Bu depo, bir dronun kontrol edilmesine yönelik temel yazılım bileşenlerini içerir. Görev mantığı, uçuş kontrolcüleri ve çeşitli yardımcı modülleri kapsar.

## Proje Yapısı

Proje aşağıdaki klasör yapısına sahiptir:

* **controllers**: Drone için farklı kontrolcü modüllerini içerir.
  * `xbee_controller.py`: XBee modülü üzerinden iletişimi yönetir.
  * `mavsdk_controller.py`: MAVSDK aracılığıyla drone ile haberleşmeyi sağlar.
  * `drone_controller.py`: Sık kullanılan dron işlemlerini step kontrol mekanizmasına uygun şekilde barındırır.
  * `offboard_controller.py`: PID ve APF işlemlerini içerir ve Offboard modu ile alakalı işlemleri barındırır.
  * `step_controller.py`: Adım tabanlı hareketleri yöneten kontrol mekanizması.

* **core**: Temel bileşenleri barındırır.
  * `drone.py`: Dronun tanımlandığı ana modüldür, diğer modüller bu modül aracılığıyla drona erişebilir.
  * `mission.py`: Görevler için temel sınıfları ve mantığı tanımlar.

* **missions**: Belirli görev uygulamalarını içerir.
  * `ucus_kanit.py`: Uçuş kanıt videosunun kodlarını barındırır.

* **utils**: Sistemin farklı bölümleri tarafından kullanılan yardımcı modüller.
  * `apf.py`: Yapay Potansiyel Alan yöntemiyle engel kaçınma algoritması.
  * `pid.py`: PID ile hedef noktaya ilerleme algoritması.
  * `socket_communication.py`: İletişim bağlantılarını socket ile sanal olarak simüle eder.
  * `formation_utilities.py`: Formasyon vb. işlemler için gerekli ortak fonksiyonları (lat_lon -> Metre vb.) barındırır.

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

### Modül Çalıştırma

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