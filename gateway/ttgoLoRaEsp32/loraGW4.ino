/*
 * V1   : Mofification pour prise de mesures de champs radio depuis la puce lora
 * V1a  :Test de modification des topic MQTT Entrant pour prise en compte d'information pour la passerelle avec une converstion string char
 * V1b  : OK  : test en passant le char topic en string pour réaliser la commparaison des topic pour gestion FONCTIONNE
 * V1c  :     : Ajout de la gestion du topic param et cmd pour passer des paramètre et des commandes à l'équipement
 * V1d  : OK  : Supression des : dans le topic MQTT
 *      : OK  : Ajout de l'envoi du rssi snr et offset par mqtt
 * V1e  : OK  : Ajout du 55 comme synchro et identification du reseau sur les com LoRa
 *      : OK  : Encodage de Kevin integré
 *      : A1  : Filtrage si le node n'est pas detinataire ou que ce n'est pas du BroadCast ATTENTION VOIR POUR LA FONCTION RELAIS
 *      : A2  : Ajout du type dans la Trame                                                ATTENTION VOIR LES TYPE D ENODE EN INCLUANT ALIMENTATIONS OU PAS
 * V2   : OK  : En restant sur la même base avec les mêmes questions a résoudre que la V1 Nettoyage de la V1e pour continuer
 *      : OK  : Abonnement au topic des commandes
 * V2a  : OK  : Utilisation de preference pour sauvegarder les réglages de fréquence
 * V2b  :     : Supression de la variable string dont je ne vois pas l'utilisé dans les settings sauvegardées en EEPROM
 * V2d  : OK  : Calibrage de la passerelle par moyennage des decalages des nodes de premiers niveau à porté de la passerelle.
 * V2e  : OK  : Ajout du lancement automatique du calibrage automatique des nodes à portés une fois la passerelle réglée.
 * V2f  : OK  : Modification pour ajout de function pour reduire le nb de ligne

*/

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Libraries Other
//#define EEPROM_SIZE 128
#include <WiFi.h>
#include <ESP32Ping.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//#define BAND 868E6      //433E6 for Asia 866E6 for Europe 915E6 for North America
#define BAND 868100000    //433E6 for Asia 866E6 for Europe 915E6 for North America
long NBAND = 868100000;   //Fréquence de base

//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
const int wifiAddr = 10;
char  CrLf[2] = {0x0D,0x0A};
String ssids_array[50];
String network_string;
char charBuf[50];
int scanWifi;
String MQTTserver;
String MQTTport;
String MQTT_USER;
String MQTT_PASSWORD;
String receivedData;
//variable pour LoRa                          
String LoRaId;                          //Identité du node, son adresse sans les :
String LoRaBr = "FFFFFFFFFFFF";         //Identité du broadcast
String LoRaDest;                        //Identité du destinataire
String LoRaSend;
String LoRaData;
String LoRaNumMes = "00";               //Numéro du message transmis par ce node
String LoRaDevTyp = "00";               //Type de Node ; 00 Gateway

//variables de réglages en fréquence
long LoRaSyTemp = 0 ;                   //temps d'attente de réponse de tous les nodes à portée lors de la synchro par réponse
long GWCorrection =0;                   //Valeur de la correction en fréquence de la gateway
int NodeCpt = 0;                        //Nombre de node de premier niveau ayant répondu pour la calibration de la gateway
boolean LoFlSyAl = false;               //si ce flag est levé alors c'est une synchro entre node.

//Variable propre au Node
String NodeId;
String BLEreceiv;
int flagBLE;

//variable pour NetWorkDyscovery

// See the following for generating UUIDs: https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

Preferences preferences;
WiFiClient wifiClient;
PubSubClient client(wifiClient);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("reco");
      deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
      Serial.println("deco");
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic){
    std::string value = pCharacteristic->getValue();
    if(value.length() > 0){
      Serial.print("Rx : ");
      Serial.println(value.c_str());
      BLEreceiv = value.c_str();
      flagBLE = 1;
      
      if(BLEreceiv.startsWith("scan,")){ 
        Serial.println("ok test");       
        scanWifi = 1;      
      }
      
      if(BLEreceiv.startsWith("55,")){
        Serial.print("start config : ");
        writeString(wifiAddr, value.c_str()); 
        hard_restart();     
      }
    }
  }
  
  void writeString(int add, String data){         //ecriture et fermeture de l'acces en mémoire flash
    preferences.putString("Data", data);
    Serial.println("Network Credentials have been Saved");
    preferences.end();
  }

  void hard_restart() {
    esp_task_wdt_init(1,true);
    esp_task_wdt_add(NULL);
    while(true);
  }
};

void reconnect() {                                 //Connexion et abonnement aux topics d'écoute du serveur MQTT
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    char charBuff1[MQTT_USER.length()+1];
    char charBuff2[MQTT_PASSWORD.length()+1];
    MQTT_USER.toCharArray(charBuff1,MQTT_USER.length()+1);
    MQTT_PASSWORD.toCharArray(charBuff2,MQTT_PASSWORD.length()+1);
    // Attempt to connect
    if (client.connect(clientId.c_str(),charBuff1,charBuff2)) {
      Serial.println("connected");
      // ... and resubscribe
      String stringOne = NodeId;
      String stringTwo;
      //Inscription au topic pour transmission lora
      stringTwo = stringOne + "/gateway/rx";
      char charBuf[stringTwo.length()+1];
      stringTwo.toCharArray(charBuf, stringTwo.length()+1);
      client.subscribe(charBuf);

      //Inscription au topic des commandes
      stringTwo = stringOne + "/gateway/cmd";
      char charBufCmd[stringTwo.length()+1];
      stringTwo.toCharArray(charBufCmd, stringTwo.length()+1);
      client.subscribe(charBufCmd);
    
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void wifiToBLE(){
  WiFi.mode(WIFI_STA);
  // WiFi.scanNetworks will return the number of networks found
  delay(500);
  int n =  WiFi.scanNetworks();
  if (n == 0) {
    pCharacteristic->setValue("no networks found .");
    pCharacteristic->notify();
    pCharacteristic->setValue(CrLf);
    pCharacteristic->notify();
  } else {
    pCharacteristic->setValue("networks found .");
    pCharacteristic->notify();
    pCharacteristic->setValue(CrLf);
    pCharacteristic->notify();
    delay(500);
    for (int i = 0; i < n; ++i) {
      ssids_array[i + 1] = WiFi.SSID(i);
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(ssids_array[i + 1]);
      network_string = i + 1;
      network_string = network_string + ": " + WiFi.SSID(i) + " (RSSI:" + WiFi.RSSI(i) + ")";
      network_string.toCharArray(charBuf, network_string.length()+1);
      pCharacteristic->setValue(charBuf);
      pCharacteristic->notify();
      pCharacteristic->setValue(CrLf);
      pCharacteristic->notify();
    }
  }
}

void setup() {     
  scanWifi = 0;      
  flagBLE = 0;
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);

  preferences.begin("Credentials", false);            //Ouverture de la mise en memoire flash de "credential"
  receivedData = preferences.getString("Data", "");   //Récupération de la chaine de type string ayant pour clef "data"
  Serial.print("Read Data:");
  Serial.println(receivedData); 
   
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  
  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA duplex 115200bd ");
  display.display();
  
  Serial.println("LoRa duplex Test");
  
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  //Freq adjust Lora
  preferences.begin("frequence", true);
  String NBANDstr = preferences.getString("frequence", "");
  NBAND = atol(NBANDstr.c_str());
  Serial.print("Frequence utilisée: ");
  Serial.println(NBAND);
  if( NBAND > (BAND + 30000) || NBAND < (BAND - 30000)){
    NBAND = BAND;
    Serial.print("Mauvaise Frequence retour origine : ");
    Serial.println(NBAND);
  }
  if (!LoRa.begin(NBAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRaFix();
  display.setCursor(0,10);
  display.println("LoRa Initializing OK!");
  display.display();
  delay(500);

  wifiTask();  
  bleTask();

  //Definition de l'identité du node
  NodeId = WiFi.macAddress();
  NodeId.replace(":", "");      //supression des : de l'adresse mac
  Serial.println("identité du node :"+NodeId);
  LoRaDest = LoRaBr;            //Destinataire par défaut est le broadcast
  
  char charBuff3[MQTTserver.length()+1];
  MQTTserver.toCharArray(charBuff3,MQTTserver.length()+1);
  client.setServer(charBuff3, MQTTport.toInt());
  client.setCallback(callback);
  reconnect();
  delay (1000);
  publishSerialData("OK Started");
}
//------------------------------------------------------------------
void loop(){

  //try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {                               //received a packet by lora
    received();
  }
  
  while (Serial.available()) // Attendre l'entré de l'utilisateur
  {    
    delay(10);
    LoRaSend = Serial.readString() ;
    //Serial.println (LoraSend);
    transmittion();    
  }

  // Un Calibrage de la passerelle est lancé ?
  if (LoFlSyAl == true){  //Si le flag est true calibrage locale actif
    long now = millis();
    if (now - LoRaSyTemp > 60000) { //si pas de réponse depuis 20 secondes (20 000)
      //traitement des resulats
      Serial.println("- Calibrage de la Passerelle démarré");
      if(GWCorrection > 0){
        Serial.print("- Correction de :");
        Serial.print(GWCorrection);
        Serial.println(" Hz à apporter.");
        calibration(GWCorrection);
        Serial.println("- Calibrage local terminé");
        SynchroAllNode();
        Serial.println("- Calibrage des nodes terminés");
      }
      LoFlSyAl= false;    //Calibrage terminé
    }  
  }

  if (scanWifi == 1){
    wifiToBLE(); 
    scanWifi = 0;    
  }

  if (flagBLE == 1){
    LoRaSend = BLEreceiv ;
    transmittion();       
    flagBLE = 0;
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

  client.loop();
}

//------------------------------------------------------------------------------------------------------------------------------------

void bleTask(){
  // Create the BLE Device
  BLEDevice::init("ESP32 LoRa GW"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
                      
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Attente connection client BLE...");
}

String getValue(String data, char separator, int index){
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found <=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
      found++;
      strIndex[0] = strIndex[1]+1;
      strIndex[1] = (i==maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void wifiTask() {
  if(receivedData.length() > 0){
    String testTrame = getValue(receivedData, ',', 0);
    String wifiName = getValue(receivedData, ',', 1);
    String wifiPassword = getValue(receivedData, ',', 2);
    MQTTserver = getValue(receivedData, ',', 3);
    MQTTport = getValue(receivedData, ',', 4);
    MQTT_USER = getValue(receivedData, ',', 5);
    MQTT_PASSWORD = getValue(receivedData, ',', 6);

    if(wifiName.length() > 0 && wifiPassword.length() > 0){
      Serial.print("WifiName : ");
      Serial.println(wifiName);

      Serial.print("wifiPassword : ");
      Serial.println(wifiPassword);

      Serial.print("MQTTserver : ");
      Serial.println(MQTTserver);

      Serial.print("MQTTport : ");
      Serial.println(MQTTport);

      Serial.print("MQTT_USER : ");
      Serial.println(MQTT_USER);

      Serial.print("MQTT_PASSWORD : ");
      Serial.println(MQTT_PASSWORD);

      WiFi.begin(wifiName.c_str(), wifiPassword.c_str());
      Serial.print("Connecting to Wifi");
      while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(1000);
      }
      Serial.println();
      Serial.print("Connected with IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("Ping Host: ");
      Serial.println(MQTTserver);
      //Serial.println(remote_host);
      MQTTserver.toCharArray(charBuf, MQTTserver.length()+1);
      if(Ping.ping(charBuf)){
        Serial.println("Success!!");
      }else{
        Serial.println("ERROR!!");
      }
      
    }
  } 
}

/*
 * -------------
 * Les fonctions
 * -------------
 */
long stringToLong(String s)           //convertion de String en Long
{
    char arr[12];
    s.toCharArray(arr, sizeof(arr));
    return atol(arr);
}
void SynchroAllNode (){
  LoRaDest = LoRaBr;                //le destinataire est broadcast
  LoRaSend = "cal";
  LoRaNumMes = "00";
  transmittion();
  Serial.println("-Interrogation des nodes à portée lancée");
}
/*
 * -------------------------------------------
 * Les fonction pour LoRa et sont exploitation
 * -------------------------------------------
 */
void received(){  //Lora : Reception
  Serial.println("- - - received LoRa packet ");
  
  while (LoRa.available()) {
    LoRaData = LoRa.readString();
    Serial.println(LoRaData);
  }
  String synch = LoRaData.substring(0, 2);
  Serial.print("- synchro ");
  Serial.println(synch);
  if( synch == "55"){
    Serial.println("- - telegrame reçu ok pour décodage "); 
    String nbtxt = LoRaData.substring(2, 4);
    Serial.print("- numero du telegram "); 
    Serial.println(nbtxt);
    String destinataire = LoRaData.substring(4, 16);
    //appel individuel ou broadcast
    if(destinataire != NodeId && destinataire != "FFFFFFFFFFFF"){
      Serial.println("- Message non destiné à ce node");
      //ATTENTION -- voir comment gérer le relayage
      return;
    }
    Serial.print("- destinataire "); 
    Serial.println(destinataire);
    String emetteur = LoRaData.substring(16, 28);
    Serial.print("- emetteur "); 
    Serial.println(emetteur);
    String typeDevice= LoRaData.substring(28, 30);
    Serial.print("- type device "); 
    Serial.println(typeDevice);
    String nivBatt= LoRaData.substring(30, 32);
    Serial.print("- alim "); 
    Serial.println(nivBatt);
    String nbData= LoRaData.substring(32, 34);
    Serial.print("- nb de data "); 
    Serial.println(nbData);
    String lotData= LoRaData.substring(34, 34 + (2*nbData.toInt()));
    Serial.print("- data:"); 
    Serial.print(lotData);
    Serial.println(":");
    
    //print RSSI of packet
    int rssi = LoRa.packetRssi();
    int snr = LoRa.packetSnr();
    int freqError =  LoRa.packetFrequencyError();
    Serial.print("- with RSSI ");    
    Serial.print(rssi);
    Serial.println("dBm"); 
    Serial.print("- SNR ");    
    Serial.println(snr);
    Serial.print("- FrequencyError ");    
    Serial.print(freqError);
    Serial.println(" HZ ");   

    // Display information
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("LORA DUPLEX");
    display.setCursor(0,10);
    display.print("Receiv packet:");
    display.setCursor(87,10);
    display.print(rssi);
    display.print("dB");
    display.setCursor(0,20);
    display.print(LoRaData);
    display.display(); 

    //Publication sur MQTT
    char charBuf[LoRaData.length()+1];
    LoRaData.toCharArray(charBuf, LoRaData.length()+1);
    publishSerialData(charBuf);  
    pubDataLora( lotData, "data", emetteur);
    pubDataLora( nivBatt, "alim", emetteur);
    pubDataLora( nbtxt, "nbTelegram", emetteur);
    pubDataLora( nbData, "nbdata", emetteur);
    pubDataLora( typeDevice, "type", emetteur);
    pubDataLora( destinataire, "destinataire", emetteur);
    pubDataLora( String(rssi), "rssi", emetteur);
    pubDataLora( String(snr), "snr", emetteur);
    pubDataLora( String(freqError), "ferror", emetteur);

    //BLE ?
    if (deviceConnected) {      
      pCharacteristic->setValue(charBuf);
      pCharacteristic->notify();
    }

    //Gestion du payload Lora
    if(lotData == "cal"){       //Demande de calibration en fréquence simple envoyée depuis la passerelle
      calibration(freqError);
    }

    if(lotData == "calall"){    //Demande de calibration en fréquence avec un calcul de la moyenne des ecarts entre la passereel et les nodes à porté.
      LoFlSyAl = true ;                 //Mise a jour du calibrage activé
      Serial.println("- - - Calibration demandée ");
      Serial.println(" - Préparation du message de réponse");
      LoRaDest = emetteur;              //le destinataire est l'emetteur du message (la passerelle)
      LoRaSend = "freqRep";
      LoRaSend += freqError;
      LoRaNumMes = "04";                //on ne passe pas par les relais
      Serial.println(" - Attente avant envoi");
      int tmps = random(10, 200);
      Serial.print("facteur de multiplicateur temps avant réponse: ");
      Serial.println(tmps);
      delay(100*tmps);
      Serial.println(" - Envoi Mesures freq");
      transmittion();
      Serial.println(" - Attente message de correction frequence");
    }
      
    String tmp = lotData.substring(0, 7);   // Gestion de la reponse lors de la demande de calibrage de la passerelle par rapport au nodes à porté
    if((tmp == "freqRep") && (LoFlSyAl == true)){             //Réponse du décalage en fréquence du node pour la passerelle
      Serial.println("- - - Calibration par moyenne des nodes de premier niveau réponse");
      Serial.print("- Décalage de fréquence du node: ");
      Serial.println(emetteur);
      String tmp2 = lotData.substring(7, lotData.length());   //Récupération de la fréquence renvoyée
      long decalage = stringToLong(tmp2);                     //passage en type long
      Serial.print("- Decalage renvoyé: ");
      Serial.print(tmp2);
      Serial.println(" Hz");
      GWCorrection = (GWCorrection - decalage)/2;
      Serial.print("- Nouvelle correction à apporter: ");
      Serial.print(GWCorrection);
      Serial.print(" Hz");
      NodeCpt++;
      Serial.print("- Nombre de Node ayant répondu: ");
      Serial.println(NodeCpt);
      Serial.println("- - - Calibration par moyenne des nodes de premier niveau réponse end");
    }
  
  }else{
    Serial.println("- - telegram provient de reception parasite"); 
  }
  Serial.println("- - - received LoRa packet end");
}
 
 void transmittion(){                                          //Transmission par Lora
  Serial.println("- - Transmission");
  Serial.println("Telegram à envoyer" + LoRaSend);
  telegramEncode();                                           //mise en forme des données à envoyer en LoRa
  Serial.println("LoRaSend aprés encodage: " + LoRaSend);
  LoRa.beginPacket();
  LoRa.print(LoRaSend);
  LoRa.endPacket();
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("LORA DUPLEX");
  display.setCursor(0,10);
  display.print("Transmit packet:");
  display.setCursor(0,20);
  display.print(LoRaSend);
  display.display();
  Serial.println("- - Transmission");
}

void LoRaFix(){
  LoRa.setTxPower(10);                //Supported values are 2 to 20 for PA_OUTPUT_PA_BOOST_PIN, and 0 to 14 for PA_OUTPUT_RFO_PIN.  LoRa.setTxPower(txPower, outputPin);
  LoRa.setSpreadingFactor(12);        //Supported values are between 6 and 12. If a spreading factor of 6 is set, implicit header mode must be used to transmit and receive packets.
  LoRa.setSignalBandwidth(125E3);     //signalBandwidth - signal bandwidth in Hz, defaults to 125E3.Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3.
  LoRa.enableCrc();                   //LoRa.disableCrc();
  //LoRa.setGain(6);                  //Supported values are between 0 and 6. If gain is 0, AGC will be enabled and LNA gain will not be used. Else if gain is from 1 to 6, AGC will be disabled and LNA gain will be used.
  Serial.println("LoRa Initializing OK!");
}

void telegramEncode(){
  String telegram;
  Serial.println("telegram: " + LoRaSend);
  telegram = "55";                                          //55 synchro et identification du reseau
  Serial.println("telegram + Synchro: " + telegram);
  telegram += LoRaNumMes;                                   //Numéro du message
  Serial.println("telegram + Numéro du message: " + telegram);
  telegram += LoRaDest;                                     //Destinataire
  Serial.println("telegram + Destinataire: " + telegram);
  telegram += NodeId;                                       //ID emetteur     ------ATTENTION -------DANS LE CAS D'UN RELAIS IL VA FALLOIR GERER
  Serial.println("telegram + emetteur: " + telegram);
  telegram += LoRaDevTyp;                                   //Type de node qui transmet ; 00: Gateway
  Serial.println("telegram + Type de node emetteur: " + telegram);
  /*
   * ATTENTION VOIR COMMENT GERER LES INFOS SUIVANT LE DEVICE TYPE Car la Gateway n'a pas de raison d'être sur batterie.
   */
  telegram += "00";                                         //Niveau de la batterie
  Serial.println("telegram + niveau batterie: " + telegram);
  int nbData = LoRaSend.length()+1;
  if(nbData <10){telegram +="0";}
  telegram += String(nbData);
  Serial.println("telegram + longueur de LoRaData" + telegram);
  telegram += LoRaSend;
  Serial.println("telegram + LoRaData" + telegram);
  LoRaSend = telegram;
}
void calibration(long correction){ //Calibration interne en fonction de l'oscillateur                     
  Serial.print("- - Calibration demandée ");   
  NBAND = NBAND - correction;
  Serial.print("- Nouvelle fréquence programmée: ");     
  Serial.print(String(NBAND));
  Serial.println(" MHZ ");    
  if (!LoRa.begin(NBAND)) {
    Serial.println("- Starting LoRa failed!");
    while (1);
  }
  preferences.putString("frequence", String(NBAND));
  delay(10);
  String NBANDstr = preferences.getString("frequence", "");
  NBAND = atol(NBANDstr.c_str());
  Serial.print("- Nouvelle fréquence corrigée lue en EEPROM: ");
  Serial.println(NBAND); 
  LoRaFix();
  delay(100);
  Serial.println("- Modification fréquence prise en compte.");
  Serial.print("- - Calibration demandée end ");
}
/*
 * --------------------------------
 * Reception depuis le brocker MQTT
 * --------------------------------
 */

void callback(char* topic, byte *payload, unsigned int length) {
  String STopic = String(topic);        //conversion en string du topic pour traitement
  //String Tbase = WiFi.macAddress();   //topic de base de la passerelle
  String Tbase = NodeId;                //topic de base de la passerelle
  Tbase += "/gateway/";
  
  Serial.println("- - - New message from broker");
  Serial.print("- Topic:");
  Serial.println(topic);
  
  //topic pour transmission Lora
  if(STopic==Tbase + "rx"){           
    Serial.println("- - Topic MQTT pour transmission");
    Serial.print("- Data:");  
    Serial.write(payload, length);
    Serial.println("");
    LoRaSend = "";                     //vidange de la variable
    for (int i = 0; i < length; i++) {
      LoRaSend += ((char)payload[i]);
    }
    transmittion();
    Serial.println("- - Topic MQTT pour transmission end");
  }
  
  //topic pour cmd
  if(STopic==Tbase + "cmd"){
    /*
    * Le Topic NodeId
    * json qui à la forme ;
    * {"cal":true} //pour lancer une synchronisation depuis la Gateway
    * {"calall":true} //pour lancer une synchronisation avce une moyenne du decallage entre la passerelle et les node à son premier niveau de porté
    */
    Serial.println("- - - Topic pour cmd");
    Serial.write(payload, length);
    DynamicJsonDocument doc(512);        //penser a ajuster la taille de la memoire alloué
    deserializeJson(doc, payload);
    boolean cal = doc["cal"];
    boolean calAll = doc["calall"];
    
    // Commande de calibrage
    if(cal == true){
      Serial.println("- - - Commande de calibration simple par rapport à la passerelle des nodes du reseau");
      SynchroAllNode();
      /*
      LoRaDest = LoRaBr;                //le destinataire est broadcast
      LoRaSend = "cal";
      LoRaNumMes = "00";
      transmittion();
      Serial.println("-Interrogation des nodes à portée lancée");
      */
      Serial.println("- - - Commande de calibration simple par rapport à la passerelle des nodes du reseau end");
    }
    //Commande de calibrage entre chaque node à porté de la passerelle
    if(calAll == true){
      Serial.println("- - Commande de calibration complete avec tout les node a portée directe de la passerelle");
      LoRaDest = LoRaBr;                //le destinataire est broadcast
      LoRaNumMes = "04";                //pas de répétion de message on aligne par rapport au premier niveau de node
      LoRaSend = "calall";
      transmittion();
      Serial.println("- Interrogation des nodes à portée lancée");
      LoFlSyAl = true;
      Serial.print("- Flag LoFlSyAl: ");
      Serial.println(LoFlSyAl);
      LoRaSyTemp = millis();            //On fait partir la tempo de limitation d etemps pour la réponse des nodes
      NodeCpt = 0;                      //mise à zéro du compteur de node qui répondent
      Serial.println("- - Commande de calibration complet avec tout les node a portée directe de la passerelle end");
      }
    
  }
  Serial.println("-------new message from broker end-----");
}

/*
 * --------------------------------
 * Publication vers le brocker MQTT
 * --------------------------------
 */
void publishSerialData(char *serialData){                                        //Publication sur NodeId/tx
  if (!client.connected()) {
    reconnect();
  }
  String topic = NodeId;
  topic += "/gateway/tx";
  char charBuf[topic.length()+1];
  topic.toCharArray(charBuf, topic.length()+1);
  client.publish(charBuf, serialData);
}

void publishStatus(char *serialData){                                             //Publication sur NodeId/status
  if (!client.connected()) {
    reconnect();
  }
  String topic = NodeId;
  topic += "/gateway/status";
  char charBuf[topic.length()+1];
  topic.toCharArray(charBuf, topic.length()+1);//-
  client.publish(charBuf, serialData);
}

void pubDataLora(String dataLora, String payloadLora, String emetteurLora){       //Permet de publier les informations de reception LoRa vers MQTT
  //String stringTwo = WiFi.macAddress();
  String stringTwo = NodeId;
  stringTwo += "/";
  stringTwo += emetteurLora;
  stringTwo += "/";
  stringTwo += payloadLora;
  char dataBuf[stringTwo.length()+1];
  stringTwo.toCharArray(dataBuf, stringTwo.length()+1);
  char dataCBuf[dataLora.length()+1];
  dataLora.toCharArray(dataCBuf, dataLora.length()+1);
  client.publish(dataBuf, dataCBuf);
}
