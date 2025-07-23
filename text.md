# Uçuş Kanıt Yazılım Videosu

## Dronlar Üzerinde Çalışan Yazılım (drone-core)

### Yazılımın Modüler Yapısı ve Modüllerin Amaçları
Dronlar üzerindeki bilgisayarlarda çalışması için geliştirdiğimiz bu yazılımın birkaç temel yapısı vardır:
- MAVSDK ve XBee Controller Sınıfları:
	
	Bu iki sınıf isimlerinden de anlaşılabileceği üzere bütün MAVSDK ve XBee bağlantı mantığını içerilerinde bulundurur ve bu yapıların diğer görev ve kodlarda tekrar kullanılabilmesini sağlar.

	Bu sayede kod yapımız temiz ve tekrardan uzak kalır.
- Drone Controller Sınıfı:
	
	Dron donanımı üzerinde işlemler yapılmasına olanak sağlayan, dron ile ilgili tüm işlemleri içerisinde barındıran kontrolcü sınıfıdır.

	Amacı gereği içerisinde ortalama bir görevde sık kullanılan işlemleri hazır olarak 
bulundurur ve görev kodlarının okunabilir kalmasını sağlar.
- Step Controller (Adım Kontrolcüsü) Sınıfı:
	
	MAVSDK kütüphanesinin yapısı gereği asenkron işlemleri await ile bekletilmesine rağmen komut gönderildiği gibi tamamlandı olarak sayıp kodun sonraki satıra geçmesine izin verdiği için geliştirilmiş olan modüldür. Aynı anda birden fazla komutun drona gönderilmesinin önüne geçer ve görev adımlarının olması gereken sırada çalışmasını sağlar.

	Kontrolcüye girilen her adımın bir bitiş şartı ve opsiyonel olarak çalışma öncesi şartı vardır. Bu şartlar sayesinde dronun yeterli irtifaya gelmeden başka komut alması gibi durumların önüne geçer. Aynı zamanda adımların zorunluluk durumuna göre belirli bir süre içerisinde şart sağlanmazsa sonraki adıma geçmeye de olanak sağlayarak dronun bir aşamada takılı kalmasını ve görevi bitirememesinin önüne geçer.
- Mission (Görev) Sınıfı:
	
	Tüm görevler bu sınıftan türetilir ve içerisinde sadece bir görevin adım ve olay akışını barındırır, görev gereksinimlerine göre dron donanımı üzerinde komutlar da gönderebildiği gibi asıl amacı sadece olay örüntüsünü barındırarak görevlerin kolayca kodlanıp anlaşılmasını sağlamaktır. 

	Çoğu görevin ortak noktası olan ve dron donanımı üzerine komutlar göndermeyi gerektiren ortak işlemler ise DroneController sınıfı üzerindeki hazır fonksyionlarla yapılarak görev kodları temiz ve karmaşadan uzak tutulur.

	Aynı zamanda görev parametrelerinin temiz bir şekilde girilmesine ve görev içerisinde kullanılmasına olanak sağlar.

### Ortalama Bir Görev Süresince Kullanılan Fonksyionlar
- Bir görev başladığında önce XBee ve MAVSDK bağlantılarının stabilize olması ve doğru telemetri verilerinin gelmesi beklenir.

- Telemetri verileri geldiğinde ise XBee yayını başlar ve görev adımları birer birer belleğe alındıktan sonra adımlar çalıştırılır

- Neredeyse her görevin birkaç ortak fonksiyonu bulunur, bunlardan bazısı:
	- Dronun konumu hiç bir hareket yapmadan, görevin başlangıcında hafızaya kaydedilir ve aynı zamanda XBee yayını arkaplanda sürekli devam edecek şekilde başlatılır
	- Dron arm edilir
	- Takeoff komutu verilir ve dron belirlenen irtifaya çıkar
	- Offboard moduna geçilir
	- Arkaplanda sürekli çalışacak olan offboard kontrolcüsü aktifleştirilir ve PID ve APF ortak olarak devreye girerek hedef konum güncellemelerini bir döngüde bekler
	Konum değiştikçe yeni konuma doğru hareket ederler
	- Göreve özel işlemler yapılır
	- Land ve Disarm komutları verilir
Ortak fonksiyonlar kendileri ile alakalı modüllerde önceden tanımlı olarak bulundukları için görev yapısı olabildiğince sadece ve olay örgüsüne odaklı olarak kalır.
---
	Not:
	Görev kodlarını ve bazı diğer fonksiyonlarını ROS1 ile çalışan eski kodlarımız üzerinden geçirildiği için hala eklenmemiş olan bazı fonksiyonlar var, ancak önceki kod yapımız üzerinden aktif olarak yeni yapıya geçirmekteyiz.


## Yer Kontrol Yazılımı

### Genel Bakış
Yer kontrol yazılımı, drone sürülerini monitör etmek ve kontrol etmek için geliştirilmiş modern bir web uygulamasıdır. Svelte ve SvelteKit framework'leri kullanılarak geliştirilmiş olup, gerçek zamanlı drone telemetri verilerini görselleştirme ve sürü yönetimi özellikleri sunmaktadır.

Arayüzün temel amacı, XBee haberleşme protokolü üzerinden gelen drone verilerini web arayüzünde görselleştirmek ve operatörlerin drone sürüsünü etkili bir şekilde yönetebilmesini sağlamaktır.

### Mimari ve Teknoloji Seçimleri
Projemizin teknik altyapısında modern web teknolojileri kullanılmıştır. Frontend tarafında Svelte 5 framework'ü tercih edilmiştir çünkü compile-time optimizasyonları sayesinde çok yüksek performans sunar ve reaktif programlama yapısıyla gerçek zamanlı veri güncellemelerini çok verimli bir şekilde yönetir.

Backend haberleşmesi için Python tabanlı bir XBee Handler modülü geliştirilmiştir. Bu modül, XBee cihazlarından gelen telemetri verilerini alır, işler ve WebSocket protokolü üzerinden web arayüzüne gerçek zamanlı olarak aktarır.

Harita görselleştirme için ise MapLibre ve Deck.gl kütüphaneleri kullanılmıştır. Bu kombinasyon sayesinde 3D drone görselleştirme, yüksek performanslı katman yönetimi ve interaktif harita özellikleri sağlanmıştır.

### XBee Haberleşme ve Veri Akışı Sistemi
xbee_handler/main.py konumundaki Python kodunda bulunan XBeeListener sınıfı, seri port üzerinden XBee cihazıyla bağlantı kurar ve gelen mesajları dinler.

Sistem thread-safe bir mesaj kuyruğu kullanarak eş zamanlı veri işlemeyi güvenli bir şekilde yönetir. Decoratorlar ile bağlantı kontrolü otomatikleştirilmiş ve hata durumlarında kullanıcıya bildirilir ve bağlantı beklenir.

XBee'den gelen ham telemetri verileri JSON formatına dönüştürülür ve WebSocket sunucusu üzerinden bağlı olan tüm web istemcilerine broadcast edilir. Bu sayede arayüz tarafında websocket'e bağlanılır ve sürü ile iki taraflı iletişime geçilir.

Mesaj formatı XBee modüllerinin limitasyonlarına uygun olarak tasarlanmış olup, drone durumu, konum bilgisi, batarya seviyesi ve uçuş modları gibi kritik bilgileri içerir.

### Kullanıcı Arayüzü ve Mimarisi
Web arayüzü üç ana panel sistemiyle tasarlanmıştır. Sol tarafta "Görev Planlayıcı", ortada "Gözlem Panelleri" ve sağ tarafta "Drone Detayları" bulunur.

Gözlem panelleri dinamik bir sekme sistemi kullanır. Bu sistem sayesinde yeni gözlem sekmeleri eklemek için sadece belirli bir klasöre yeni bir Svelte bileşeni eklenmesi yeterlidir. Şu anda harita ve terminal çıktılarının bulunduğu sekmeler bulunmaktadır.

Dron verilerinin reaktif olarak yönetmek için Svelte 5'in yeni Rune sistemi kullanılmıştır. Keşfedilmiş dron durumları tüm bileşenler arasında paylaşılır ve herhangi bir değişiklik otomatik olarak ilgili arayüz bileşenlerini günceller.

Harita görselleştirmede Deck.gl'in katman sistemi kullanılarak drone'lar harita katmanı üzerinde ayrı bir katman olarak 3D modeller ile temsil edilir. Drone konumları ve durumu gerçek zamanlı olarak XBee'den iletilen veriler ile güncellenir. Ayrıca rota çizgileri, taranan alanlar ve hedef noktalar gibi ek katmanlar da dinamik olarak yönetilebilir.

### Görev Planlama ve Drone Yönetimi
Mission Planner bileşeni, drone sürüsü için görev parametrelerinin belirlenmesini sağlar. Farklı formasyon tipleri önceden tanımlanmış olup - V formasyonu, çizgi formasyonu, ok başı formasyonu seçenekleri mevcuttur.

Her formasyon için drone pozisyonları matematiksel olarak hesaplanır ve görsel olarak önizleme sağlanır. Görev tipi olarak hedefe ulaşma veya alan tarama seçenekleri bulunur. Hedef koordinatları, uçuş yüksekliği ve diğer parametreler kullanıcı dostu arayüz ile girilebilir.

Drone Detayları sekmesinde her keşfedilen drone için ayrı sekmeler dinamik olarak oluşturulur. Bu sekmelerde drone'un batarya durumu, konum bilgisi, ping değeri ve diğer telemetri verileri gerçek zamanlı olarak görüntülenir.

Harita üzerinde drone'lara tıklanarak onlara odaklanılabilir. Ping hesaplaması drone'dan gelen timestamp ile mevcut sistem zamanı arasındaki fark hesaplanarak yapılır.