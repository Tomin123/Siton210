/*
SOLAR INVERTER 2 - SITON 210

(c) Tomas Nevrela; tnweb.tode.cz


Program stridace pro fotovoltaicke panely a ohrev bojleru.
Prevadi DC proud z FV panelu na stridavy obdelnikovy s promennou stridou.
Vyhledava bod nejvyssiho vykonu fv panelu, zobrazeni hodnot na I2C LCD
pres expander PCF8574
mereni napeti a proudu, tlacitka pro rucni zmenu stridy menice
rucni a automaticky rezim, vypocet vyroby EE, pri zvyseni vyroby o 0,1kWh
ulozi vyrobu do EEprom, komunikace po RS485, mereni teploty bojleru.

Poznamka:
Pri pouziti bootloaderu musi byt v Arduinu novejsi verze bootloaderu (optiboot)
jinak watchdog nefunguje spravne.
Pouzita nestandartni inicializace dvou typu komunikaci na jedno sw seriove
rozhrani.

*/
#define	ksVersionNumber		" sw.v8.2.00     "
/*

upravy (c) Tomas Chvatal
verze 8.2.0
 - vracena podpora Komunikace()
verze 8.1.0
 - zlepseno rozliseni/vypocet PROUDU
 - zlepseno rozliseni vykonu - prom. vykon je nyni float (drive int) 
verze 8.0.0
 - zlepsena ochrana eeprom - zapis vyroby do eeprom se deje pouze pokud se neco vyrobilo (v noci ne)
 - zvysena vyteznost pri slabsim osvetleni - zpomaleno vandrovani do stran MPPT kolena 
 - BEZ komunikace !
 

Verze 5.4.27(3f)
file:Solar_inverter54.27.ino
kompilace Arduino 1.8.5
15.2.2022
Zmeny:
4.2 - opravena chyba nacitani vyroby
4.3 - opraveny adresy v eeprom po 90000 zapisech
- automaticka inicializace Eeprom po prvnim nahrani programu
4.4 - zvysena zmena pwm na -3 a +4
- zvysen cas prepnuti z ruc. do aut. rezimu na 10s
4.5 - tlacitka pres analogovy vstup
- mereni teploty bojleru
- periodicka kontrola VA krivky
- omezeni proudu na 10A
4.5a- jumper na D5 pro vypnuti funkce kontroly VA krivky
4.6 - nova verze DPS V2C s nadproudovou ochranou a budici IR2104
4.7 - svorkovy vstup pro povoleni provozu menice verze DPS V2D
4.8 - kazdych 60min. se provede zapis vyroby do EEprom
4.9 - Setting menu, nastaveni max. teploty, max. vykonu,
krivky, perioda testu VA krivky, ID menice, max.hodnoty + moznost
nulovat max. hodnoty
5.0 - pridana komunikace MODBUS
- run LED behu programu
- pridana moznost nastaveni celkove vyroby pri zapnuti
5.1 - rizeni podsviceni LCD po I2C pres Attinyx5 adresa 0x25
5.1a- blokovani chodu menice pri nastavovani celkové vyroby kWh
5.2 - osetreni odpojeneho termocidla; max. podsviceni pri stisku tlacitka
5.3 - Oprava konfliktu reset watchdogu a komunikace Modbus
do menu pridana polozka tovarni reset - maze vyrobu a nastaveni
5.4 - Zmena principu resetu menice po tovarnim resetu pomoci watchdogu

*/


#include <SoftEasyTransfer.h> //https://github.com/madsci1016/Arduino-SoftEasyTransfer
#include <SoftwareSerial.h>
#include <ModbusRtu.h> //https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
#include <EEPROM.h>
#include "EEPROMAnything.h" //https://github.com/collin80/EEPROMAnything
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>
#include "myTimer.h"
//#include "ma.h"
#include <Arduino.h>

// Settings
#define		klSkipIntro			false				// pro rychlejsi debug a ladeni [false / true]
#define		kSet_MaxProud		12
#define		klLedBlik			true				// ma blikat LED v prav.cas.intervalech

//#define		kSet_MaxProud		10
#define I2C_Address		0x27 // adresa I2C prevodniku PCF8574T,PCF8574P 0x27 (PCF8574AT 0x3f)
#define AttinyAddress	0x25 // adresa Attiny desky LCD



// o kolik zvedam stridu pri cestovani nahoru
// pokud by byl krok moc maly, proudove cidlo (se svym limitnim rozlisenim a sumem) 
// by mohlo vratit MENSI hodnotu i kdyz spravne doslo k NARUSTu vykonu
#define PWMDUTY_DELTA_PLUS		3
// o kolik snizuji stridu pri cestovani dolu
#define PWMDUTY_DELTA_MINUS	2


SoftwareSerial mySerial(8, 6);					// Arduino RX - RS485 RO = 8, Arduino TX - RS485 DI=6
#define	hwSerial		Serial						// pro vysilani DEBUG info pres terminal

#define TXenableRS485 7 //RE + DE
#define Buttons  A3 //pripojene tlacitka
#define KeyExit  20 //nepouzito
#define KeyUp    30 //nahoru
#define KeyDown  40 //dolu
#define KeySel   50 //stisknuta obe tlacitka
#define KeyInv   60 //neplatna hodnota tlacitka
#define menuTimeExit 10000 //po case ukonci Menu
#define pinA		A1 //pin mereni proudu FV
#define pinV		A2 //pin mereni napeti FV
#define cidlo_B	A0 //cidlo teploty KTY81/210
#define R25			2000 //referencni odpor termocidla pri 25st C
#define mppt_pin	 5 //jumper rezerva
#define enable_pin 4 // pin povoleni provozu menice
#define ochrana_pin 11 // pin stavu nadproudove ochrany
#define ochrana_reset_pin 12 //reset nadproudove ochrany
#define R1			470000.0 //odpory delice napeti FV
#define R2			5600.0
#define maxStrida 247 //maximalni strida (nemenit!)
#define frekvence 50 //frekvence PWM vystupu (nemenit!)
#if kSet_MaxProud==12
#define Maxproud 1200 //max. proud 12.00A
#else
#define Maxproud 1000 //max. proud 10.00A
#endif

// LED na pinu 1 (TXD) nepouzivam - chci nechat HW UART pro prenos dat
//#define runLED 1  // signalizacni LED behu
#define pinLedRun				LED_BUILTIN
#define pinLedKomunik		1	//LED_BUILTIN  // signalizacni LED komunikace

// delka bufferu = casova konstanta filteru
//static unsigned int iaFiltVykon[4];
//struct ma strFiltVykon;


int R = 2200; // hodnota rezistoru v delici s termocidlem
int TeplBojl; //skutecna teplota bojleru (TUV)
byte stupen[8] = {
	  0b01100
	, 0b10010
	, 0b10010
	, 0b01100
	, 0b00000
	, 0b00000
	, 0b00000
	, 0b00000
	};
int klavesa;
byte offset, whichkey;
bool showStatus = false; //
int nodeID = 12; // cislo Emon nodu
int interval_LCD = 800; //cas obnoveni LCD
unsigned long interval_VA =  60000; //interval kontroly VA krivky
unsigned long interval_WR = 3600000; //interval zapisu vyroby - 60 min.
int rucTimeExit = 10000; //cas prepnuti z rucniho zpet do aut. rezimu v ms
int perKomunikace = 3000; // perioda odesilani dat v ms
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long previousVA = 0;
unsigned long previousWR = 0;
unsigned long casKomunikace;
unsigned long casLED;
unsigned long rucTime;
unsigned long vyrobaTime = 0;
unsigned long vyroba = 0;
unsigned long lastvyroba;
float whInc2;
bool nadproud = false;			// true, pokud se zmeri proud >= Maxproud
bool ochrana = false;			// true, pokud hw ochrana zjisti nadproud
bool zapis;
int strida = 8;
long topv;
int nap, valA, valV, vcc;
float vPow;
int napeti, proud;
float vykon;
float vykon_prev;
byte restart = 0;
int cnt = 200;						//pocet vzorku mereni
int smer = 1;						//smer zmeny stridy 1 = zvysovani 2 = snizovani
bool rucne = false;				//rezim rizeni
int addr_vyroba = 45;
int addr_suma_write = 55;
unsigned long SumaWrite;		// aktualni pocet zapisu do EEPROM
unsigned long PreviousWrite;	// predchozi hodnota zapisu do EEPROM ++90000
unsigned long FirstRun;			// hodnota kontroly prvniho spusteni programu
unsigned long menuTime = 0;
unsigned long hodnotaL;
int teplotaMax = 90;				//max teplota
int vykonMax = 2800;				//max vykon
int timeTestMPPT = 5;			//perioda kontroly krivky MPPT
int onVA = 1;						// kontrola VA krivky aktivni
int maxV = 0;						//max. namerene napeti
int maxA = 0; //max. namereny proud
int maxW = 0; //max. namereny vykon
int kalibV = 0; //kalibr. konstanta napeti
int kalibA = 0; //kalib. konstanta proudu
int KomTyp = 1; // typ komunikace 0 = Modbus RTU, 1 = EasyTransfer
int LCDbacklight = 5; // uroven podsviceni
int LCDbacklightLast; // posledni uroven podsviceni
bool set_kwh = false; // rezim nastaveni kWh

const char string_0[] PROGMEM =   "Exit"; //0
const char string_1[] PROGMEM =   "Max.teplota"; //1
const char string_2[] PROGMEM =   "Max.vykon"; //2
const char string_3[] PROGMEM =   "Perioda testu VA"; //3
const char string_4[] PROGMEM =   "ID menice/adresa"; //4
const char string_5[] PROGMEM =   "Kalibrace V"; //5
const char string_6[] PROGMEM =   "Kalibrace A"; //6
const char string_7[] PROGMEM =   "Komunikace"; //7
const char string_8[] PROGMEM =   "Podsviceni LCD"; //8
const char string_9[] PROGMEM =   "Max. hodnoty"; //9
const char string_10[] PROGMEM =  "Tovarni RESET"; //10 rezerva
const char string_11[] PROGMEM =  " "; //11 rezerva
const char string_12[] PROGMEM =  "  *NASTAVENI*   "; //12
const char string_13[] PROGMEM =  "ukladam..."; //13
const char string_14[] PROGMEM =  "Suma vyroby"; //14


PGM_P const StringTable[] PROGMEM = {
	string_0,
	string_1,
	string_2,
	string_3,
	string_4,
	string_5,
	string_6,
	string_7,
	string_8,
	string_9,
	string_10,
	string_11,
	string_12,
	string_13,
	string_14,
};


// struktura pro komunikaci
struct PayloadTX {
	byte nodeID, address, command, func;
	int data1, data2, data3, data4, data5, data6, data7, data8;
	long data9, data10, data11, data12;
};
PayloadTX emontx;   // vytvoreni instance
SoftEasyTransfer ET;

// pole dat pro modbus
unsigned int holdingdata[20] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
unsigned int cnt_sl;
Modbus slave(nodeID, 4, TXenableRS485); // slave adresa,mySoftwareSerial,RS485 enable pin

// Nastav pro LCD piny PCF8574AT a I2C adresu 0x3f(PCF8574T 0x27)
//                    addr,       en,rw,rs,d4,d5,d6,d7,bl, blpol
LiquidCrystal_I2C lcd(I2C_Address, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE );



unsigned long GetTimeElaps(myTimer tmr) {
	myTimer nTmrCurr = millis();
	if (nTmrCurr < tmr)								// preteceni timeru pres max.
		return 4294967295ul - (tmr-nTmrCurr);
	else
		return nTmrCurr - tmr;
}

unsigned long GetTimeElapsSec(myTimer tmr) {
	return GetTimeElaps(tmr) / 1000;
}

void ResetTimer(myTimer *tmr) {
	*tmr = millis();
}



//==============================================================================
//=================================SETUP========================================
//==============================================================================
void setup() {
	wdt_enable(WDTO_8S);// aktivace watchdogu na 8s
	wdt_reset(); // vynulovani watchdogu
	//==============================================================================
	//nepouzite vyvody sepnout ke GND (omezeni vlivu ruseni)
	pinMode(A6, OUTPUT);
	digitalWrite(A6, LOW);
	pinMode(A7, OUTPUT);
	digitalWrite(A7, LOW);
	pinMode(2, OUTPUT);
	digitalWrite(2, LOW);
	pinMode(3, OUTPUT);
	digitalWrite(3, LOW);
	//==============================================================================
	// Zapis vychozich hodnot do Eeprom pri prvnim spusteni programu.

	//nacte kontrolni hodnotu z konce Eeprom
	EEPROM_readAnything(1020, FirstRun);

	//pokud hodnota nesouhlasi provede se prvotni inicializace Eeprom
	if (!(FirstRun == 0x710812AA))
	{
		EEPROM_writeAnything(2, addr_vyroba);// int
		EEPROM_writeAnything(6, addr_suma_write);// int
		EEPROM_writeAnything(8, PreviousWrite);// long
		EEPROM_writeAnything(12, nodeID);// int
		EEPROM_writeAnything(14, teplotaMax);// int
		EEPROM_writeAnything(16, vykonMax);// int
		EEPROM_writeAnything(18, timeTestMPPT);// int
		EEPROM_writeAnything(20, onVA);// int
		EEPROM_writeAnything(22, maxV);// int
		EEPROM_writeAnything(24, maxA);// int
		EEPROM_writeAnything(26, maxW);// int
		EEPROM_writeAnything(28, kalibV);// int
		EEPROM_writeAnything(30, kalibA);// int
		EEPROM_writeAnything(32, KomTyp);// int
		EEPROM_writeAnything(34, LCDbacklight);// int
		EEPROM_writeAnything(addr_vyroba, vyroba);
		EEPROM_writeAnything(addr_suma_write, SumaWrite);
		//zapise kontrolni hodnotu o provedene prvotni inicializaci
		FirstRun = 0x710812AA;
		EEPROM_writeAnything(1020, FirstRun);
	}

	//==============================================================================

	EEPROM_readAnything(2, addr_vyroba); //adresa pro nacteni sumy vyroby
	EEPROM_readAnything(6, addr_suma_write); //adresa pro nacteni sumy zapisu do EEPROM
	EEPROM_readAnything(8, PreviousWrite); //predchozi hodnota poctu zapisu do EEPROM
	EEPROM_readAnything(12, nodeID);// int
	EEPROM_readAnything(14, teplotaMax);// int
	EEPROM_readAnything(16, vykonMax);// int
	EEPROM_readAnything(18, timeTestMPPT);// int
	EEPROM_readAnything(20, onVA);// int
	EEPROM_readAnything(22, maxV);// int
	EEPROM_readAnything(24, maxA);// int
	EEPROM_readAnything(26, maxW);// int
	EEPROM_readAnything(28, kalibV);// int
	EEPROM_readAnything(30, kalibA);// int
	EEPROM_readAnything(32, KomTyp);// int
	EEPROM_readAnything(34, LCDbacklight);// int
	EEPROM_readAnything(addr_vyroba, vyroba); //nacteni vyroby z aktualni adresy
	EEPROM_readAnything(addr_suma_write, SumaWrite);// nacteni sumy zapisu z akt. adresy
	lastvyroba = vyroba;

	pinMode(ochrana_pin, INPUT); //vstup stavu nadproudove ochrany
	
	pinMode(TXenableRS485, OUTPUT);
	digitalWrite(TXenableRS485, LOW);
	pinMode(pinLedKomunik, OUTPUT);
	pinMode(pinLedRun, OUTPUT);
	digitalWrite(pinLedRun, LOW);
	pinMode(mppt_pin, INPUT_PULLUP);//jumper rezerva
	pinMode(enable_pin, INPUT_PULLUP);//pin povoleni provozu menice

	hwSerial.begin(115200);
	//hwSerial.println("start");

	// softwareSerial pro RS485
	mySerial.begin(9600);
	ET.begin(details(emontx), &mySerial);// nastaveni EasyTransfer
	// Modbus
	slave.begin(&mySerial, 9600); 
	slave.setID(nodeID);						// Modbus slave adresa

	delay(60);
	lcd.begin(16, 2); // pocet znaku, pocet radku
	lcd.createChar(1, stupen); // ulozi do LCD symbol stupnu
	lcd.backlight(); // zapni podsvetleni (noBacklight -vypni)

	//----------------Test stisku tlacitka pro nastaveni vyroby------------
	klavesa = KeyScan();
	if (klavesa == KeyUp) {
		delay(2000);
		// TomCh-doplneno nove cteni klavesy - pokud i po XXms je stale klavesa drzena ...
		klavesa = KeyScan();
		if (klavesa == KeyUp) {
			showStatus = true;
			nastaveni_kwh();
		}
	}
	
	//---------------------------------------------------------------------
#if (klSkipIntro==false)
	lcd.setCursor(0, 0);
	lcd.print(" SOLAR INVERTER ");
	lcd.setCursor(0, 1);
	lcd.print("    SITON 210   ");
	delay(2500);
	lcd.clear();
	wdt_reset(); //reset watchdogu
	lcd.setCursor(0, 0);
	lcd.print(ksVersionNumber);
	lcd.setCursor(0, 1);
	lcd.print(" 06/2022        ");
	delay(2000);
	lcd.clear();
	wdt_reset(); //reset watchdogu
	
	const String line1 = " Tomas Nevrela  vylepseni    Tomas Chvatal  ";
	const String line2 = " tnweb.tode.cz     od        www.fordiag.cz ";
	for (int x=0; x<(line1.length()-15); x++) {
		lcd.setCursor(0, 0);
		lcd.print(line1.substring(x,x+15));
		lcd.setCursor(0, 1);
		lcd.print(line2.substring(x,x+15));
		if (x==0)
			delay(2000);
		else {
			delay(100);
		}
		wdt_reset();
	}
	delay(2500);
#endif


	//-----------------reset nadproudove ochrany----------------------------------
	digitalWrite(ochrana_reset_pin, HIGH);
	pinMode(ochrana_reset_pin, OUTPUT);// vystup na HIGH
	delay(10);
	pinMode(ochrana_reset_pin, INPUT);// HI Impedance
	//----------------------------------------------------------------------------

	/*FilterInit(&strFiltVykon,
		iaFiltVykon,
		sizeof(iaFiltVykon)/sizeof(iaFiltVykon[0]),
		sizeof(iaFiltVykon)/sizeof(iaFiltVykon[0])-1
		);
	ResetFilter(&strFiltVykon);	*/


}

//==============================================================================
//==============================Hlavni smycka===================================
//==============================================================================
void loop() {
	
	// cca kazdych 100ms (viz. delay() nize)

	mereni(); // mereni hodnot
	if ((digitalRead(enable_pin)) || (TeplBojl >= teplotaMax)) {
		strida = 0; // provoz menice neni povolen, nastav stridu na 0
	} else {
		rizeni(); // MPPT rizeni
		testVA (); //test VA krivky

		//------------------pusobeni nadproudove ochrany---------------------------------

		if (ochrana) {
			lcd.clear();
			lcd.setCursor(0, 0);// sloupec, radek
			lcd.print("  NADPROUDOVA  ");
			lcd.setCursor(0, 1);// sloupec, radek
			lcd.print("   OCHRANA!!   ");

			if (restart <= 4) //reset ochrany povolen 5x
			{
				strida = 8;//
				smer = 1;
				set_PWM(strida);

				for (int i = 0; i <= 15; i++) // cekej 15s
				{
					delay(1000);
					wdt_reset();
				}
				// odblokuj nadproudovou ochranu a zkus spustit menic
				digitalWrite(ochrana_reset_pin, HIGH);
				pinMode(ochrana_reset_pin, OUTPUT);// vystup na HIGH
				delay(10);
				pinMode(ochrana_reset_pin, INPUT); // vystup HI impedance
				ochrana = false; //zrus priznak aktivace ochrany
				restart++;
			}
			// pokud byla naproudova ochrana aktivovana vice nez 5x od posledndiho resetu
			// zustane vykonovy stupen zablokovany a musi se provest vypnuti a zapnuti menice
			else
			{
				strida = 0;
				set_PWM(strida);
				wdt_disable();
				while (1);
			}
		}
	}
	//------------------------------------------------------------------------------
	set_PWM(strida);
	delay(10);//
	zobrazeni();
	if (!rucne) 
		delay(100); // zpozdeni pokud neni rucni rezim
	komunikace();
	
	//------------------------------------------------------------------------------
	// zapis vyroby do EEprom po 60 min.
	currentMillis = millis();
	if ((unsigned long)(currentMillis - previousWR) >= interval_WR) {
		zapis = true; //
		previousWR = currentMillis;
	}
	//-----------------------------------------------------------------------------

	#if klLedBlik
		// signalizace behu programu
		currentMillis = millis();
		if ((unsigned long)(currentMillis - casLED) >= 1000)
		{
			digitalWrite(pinLedRun, HIGH);
			delay(30);
			digitalWrite(pinLedRun, LOW);
			casLED = currentMillis;
		}
	#endif

	//--------------------------------Zapis do EEPROM------------------------------
	// Zapis do EEPROM se provede pokud se vyroba zvysi o 100Wh
	//  posun adresy po 90000 zapisech
	if (zapis) {
		zapis = false;
		// eeprom protection
		unsigned long tmp = 0;
		EEPROM_readAnything(addr_vyroba, tmp);
		if (tmp != vyroba) {													// value changed ?
			EEPROM_writeAnything(addr_vyroba, vyroba);				// uloz vyrobu
			SumaWrite++;
			EEPROM_writeAnything(addr_suma_write, SumaWrite);		//uloz celkovy pocet zapisu
			// pokud pocet zapisu do jedne bunky presahl 90000 posun se na dalsi
			if ((SumaWrite - PreviousWrite) >= 90000) {
				addr_vyroba += 10; // posun adresy pro zapis o 10
				addr_suma_write += 10;
				// zapis aktualnich hodnot do EEPROM
				EEPROM_writeAnything(2, addr_vyroba); //uloz akt. adresu vyroby
				EEPROM_writeAnything(6, addr_suma_write); //uloz akt. adresu sumy zapisu
				PreviousWrite = SumaWrite;
				EEPROM_writeAnything(8, PreviousWrite);
				EEPROM_writeAnything(addr_vyroba, vyroba);
				EEPROM_writeAnything(addr_suma_write, SumaWrite);
			}
		}
	}

	//=============================Obsluha tlacitek==================================

	klavesa = KeyScan();
	if (klavesa == KeySel) {
		showStatus = true; 
		offset = 0;
	}
	if (showStatus) 
		mainMenu();
	if (!digitalRead(enable_pin)) //proved pokud je povolen provoz menice na vstupu enable
	{

		if (klavesa == KeyUp)
		{
			strida++;
			rucne = true; //po stisku tlacitka rezim zmeny stridy rucne
			rucTime = millis();
			delay(15);
		}
		if (strida > maxStrida) strida = maxStrida;

		if (klavesa == KeyDown)
		{
			strida--;
			rucne = true; //po stisku tlacitka rezim zmeny stridy rucne
			rucTime = millis();
			delay(15);
		}
		if (strida < 8) strida = 8;

	}
}
//==========================konec hlavni smycky=================================
//==============================================================================



//===============================Komunikace=====================================
void komunikace()
{
	if (KomTyp) {
		currentMillis = millis();
		if ((unsigned long)(currentMillis - casKomunikace >= perKomunikace))
		{
			//EasyTransfer
			// naplneni struktury dat
			emontx.data1 = napeti;
			emontx.data2 = proud;
			emontx.data3 = vykon;
			emontx.data4 = TeplBojl;
			emontx.data9 = vyroba; //vyroba ve Wh
			emontx.nodeID = nodeID; // ID solar invertoru
			emontx.command = 5; // odeslani dat bez pozadavku
			emontx.address = 15; // adresa emonHUBu
			#if klLedBlik
				digitalWrite(pinLedKomunik, HIGH);// activity LED
			#endif
			delay(15);
			digitalWrite(TXenableRS485, HIGH);//prepni prevodnik RS485 na vysilani
			ET.sendData(); // odesli data na emonHUB
			digitalWrite(TXenableRS485, LOW);
			delay(20);
			#if klLedBlik
				digitalWrite(pinLedKomunik, LOW); // activity LED
			#endif
			casKomunikace = currentMillis;
		}
	}
	else {
		//Modbus RTU
		unsigned int HighINT;
		unsigned int LowINT;
		holdingdata[0] = nodeID;
		holdingdata[1] = 0;//address
		holdingdata[2] = 0;//command
		holdingdata[3] = 0;//func
		holdingdata[4] = napeti;
		holdingdata[5] = proud;
		holdingdata[6] = vykon;
		holdingdata[7] = TeplBojl;
		holdingdata[8] = 0;
		holdingdata[9] = 0;
		holdingdata[10] = 0;
		holdingdata[11] = 0;
		//vyroba = 4567892;
		HighINT = vyroba >> 16 & 0x0000ffffl ;//vyroba;
		LowINT = vyroba & 0x0000ffffl;


		holdingdata[12] = LowINT; //vyroba LO
		holdingdata[13] = HighINT; //vyroba HI
		holdingdata[14] = 0;
		holdingdata[15] = 0;
		holdingdata[16] = 0;
		holdingdata[17] = 0;
		holdingdata[18] = 0;
		holdingdata[19] = 0;


		slave.poll( holdingdata, 20 );

		if (cnt_sl != slave.getOutCnt()) {
			#if klLedBlik
				digitalWrite(pinLedKomunik, HIGH);
				delay(40);
				digitalWrite(pinLedKomunik, LOW);
			#endif
		}
		cnt_sl = slave.getOutCnt();

	}

}




//===============================Nastaveni PWM==================================
//==============================================================================
//Nastavi casovace PWM, max. hodnota OCR1A=1235, OCR1B=1265 pro dostatecny dead
//time (120us)
//vystup na pinech D9 a D10 casove posunute o 10ms
void set_PWM(int dutyCycle)
{
	if (nadproud) { // pri nadproudu sniz stridu
		dutyCycle = dutyCycle - 25;
		if (dutyCycle <= 5) dutyCycle = 5;
		strida = dutyCycle;

		smer = 2;//smer dolu
		lcd.clear();
		lcd.setCursor(0, 0);// sloupec, radek
		lcd.print("   NADPROUD!   ");
	}
	if (dutyCycle > maxStrida) 
		dutyCycle = maxStrida;
	else if (dutyCycle < 0) 
		dutyCycle = 0;
	
	cli();
	TCCR1B = _BV(WGM13) | _BV(CS11) | _BV(CS10);// rezim 8,clock/64

	int topv = (F_CPU / (frekvence * 2 * 64));//16000000/(50*2*64)=2500
	ICR1 = topv;
	OCR1A = (dutyCycle * 5); //od 0 do 1235
	OCR1B = ( topv - (dutyCycle * 5)); //od 2500 do 1265

	DDRB |= _BV(PORTB1) | (_BV(PORTB2)); //D9(PB1), D10(PB2) nastav na vystup
	// rezim vyvodu OC1A (Arduino D9)  [set to low on ...]
	//              OC1B (Arduino D10) [set to high on ...]
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0); 
	sei();
}


//===============================Mereni=========================================
//==============================================================================
//mereni proudu,napeti a teploty, vypocet vykonu a vyroby,
//prevod potrebnych hodnot do int
void mereni()
{
	float amp, Anormal;

	wdt_reset();
	vcc = readVcc();// zmereni napajeciho napeti 5V
	vPow = vcc / 1000.0;

	unsigned long totV = 0;
	unsigned long totA = 0;
	unsigned long totT = 0;
	for (int i = 0; i < cnt; i++)
	{
		totA = totA + analogRead(pinA); //zmer proud
		totV = totV + analogRead(pinV); //zmer napeti
		totT = totT + analogRead(cidlo_B);//teplota
	}
	// fix, TomCh
	// zlepseno rozliseni mereni proudu, prom. amp je nyni float
	amp = (double)totA / cnt; // prumer ze vzorku
	nap = totV / cnt;
	totT = totT / cnt;
	// vypocet proudu - normalizuji z AD na Ampery
	Anormal = amp * (vcc / 1023.0);
	Anormal = (Anormal - (vcc / 2)); // 0856 = 8,56A
	//Anormal = 9;//###DEBUG###
	Anormal = Anormal + ((Anormal / 100) * (kalibA / 10)); //kalibrace hodnoty proudu
	//proud = (int) Avolt;
	proud = (int) roundf(Anormal);
	//proud = 920; //###DEBUG###

	if (proud <= 9) proud = 0;//kalibrace 0
	//-------------------------------------------
	if (proud >= Maxproud) {
		nadproud = true; //omezeni trvaleho proudu
	} else {
		nadproud = false;
	}
	//---------------------------------------------
	if (!digitalRead (ochrana_pin))
		ochrana = true;									// pri aktivni nadproudove ochrane nastav priznak

	float a = proud;
	float v = (nap * vPow) / 1023.0;
	float Vnormal = v / (R2 / (R1 + R2));					// vypocet napeti podle delice
	Vnormal = Vnormal + ((Vnormal / 100) * (kalibV / 10));		// kalibrace napeti
	//Vnormal = 289;//###DEBUG###

	napeti = (int) roundf(Vnormal);
	float vyk = (Vnormal * a) / 100 ;					// vypocet vykonu
	//vykon = roundf(vyk);							// okamzity vykon
	// zkousen filtr, ale neni treba, navic zanasi zaokrouhleni z FLOAT na INT
	//vykon = NextElement(&strFiltVykon, roundf(vyk)) / 100;
	vykon = vyk;										// okamzity vykon v nejlepsim rozliseni
		

/*hwSerial.print("Vnormal"); hwSerial.println(Vnormal);
hwSerial.print("a"); hwSerial.println(a);
hwSerial.print("vykon"); hwSerial.println(vykon);	*/

// pro SerialPlot
hwSerial.print("Splot:");
hwSerial.print(Vnormal); hwSerial.print("|");
hwSerial.print(a); hwSerial.print("|");
hwSerial.print(vykon); hwSerial.print("|");
hwSerial.print(strida); hwSerial.print("|");
hwSerial.println("");


	if (napeti > maxV) {
		maxV = napeti;
		EEPROM_writeAnything(22, maxV);//
	}
	if (proud > maxA) {
		maxA = proud;
		EEPROM_writeAnything(24, maxA);//
	}
	if (vykon > maxW) {
		maxW = vykon;
		EEPROM_writeAnything(26, maxW);//
	}
	//vykon = 3500;//###DEBUG###
	if (totT > 1020) // pokud je odpojené termoèidlo nastav 0 oC
	{
		totT = 437;
	}

	TeplBojl = vypocet(totT); //vypocet teploty


	//=====================Vypocet vyroby Wh====================================
	unsigned long lvyrobaTime = vyrobaTime;
	vyrobaTime = millis();
	float whInc = vykon * ((vyrobaTime - lvyrobaTime) / 3600000.0);
	whInc2 = whInc2 + whInc;
	if (whInc2 > 1.0)	{
		whInc2 = whInc2 - 1.0;
		vyroba++;
		if (vyroba >= 100000000) {			//maximalni rozsah 100000.00kWh
			vyroba = 0;
			lastvyroba = vyroba;
		}
	}
	if ((vyroba - lastvyroba) >= 100) {		 //zapis do eeprom pri zvyseni o 0.1kWh
		zapis = true;
		lastvyroba = vyroba;
	}

}
//============================================================================
// vypocte z analogove hodnoty teplotu
int vypocet(int value) {
	float ukty = value * ( vPow / 1023.0 ) ; //vypocet napeti na senzoru
	// alfa x R25
	float a = 0.007874 * R25;
	// beta x R25
	float b = 0.00001874 * R25;
	// koeficient c
	float c = R25 - R * ukty / (vPow - ukty);
	float delta = a * a - 4 * b * c;
	float delta1 = sqrt (delta);
	float x2 = (-a + delta1) / (2 * b);
	float temp1 = x2 + 25 ;
	int teplota = temp1;//prevod na int
	return teplota;
}
//=======================Zmereni napeti 5V====================================
long readVcc()
{
	long result;
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(2);
	ADCSRA |= _BV(ADSC);
	while (bit_is_set(ADCSRA, ADSC));
	result = ADCL;
	result |= ADCH << 8;
	result = 1126400L / result;
	return result;
}

//==========================Rizeni vykonu MPPT==================================
//==============================================================================
//hledani max. vykonu fotovoltaickych panelu
void rizeni() {
	static myTimer tmrRizeni;

	currentMillis = millis();
	if ((unsigned long)(currentMillis > (rucTime + rucTimeExit))) 
		rucne = false;

	if (rucne == false) {
		// pouze kazdych XXms
		if (GetTimeElaps(tmrRizeni) < 300)
			return;
		ResetTimer(&tmrRizeni);
//hwSerial.print("ted"); hwSerial.println(millis());
/*hwSerial.print(smer); hwSerial.print(" ");
hwSerial.print("vykon minuly:"); hwSerial.print(vykon_prev);
hwSerial.print("  /  vykon:"); hwSerial.print(vykon); 
*/


		if (vykon == 0) {
			strida = 24;	//pri nulovem vykonu nastav zakladni stridu a smer
			smer = 1;		//zvysovani

		} else if (vykon >= vykonMax) {
			//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
			// omezeni stridy pri max vykonu
			strida -= 4;
			if (strida < 5) {
				strida = 5;
				smer = 1;
			} else
				smer = 2;

		} else {
			// normal MPPT rizeni
			//smer nahoru

			//hwSerial.print("                                                                            ");
			//hwSerial.print(vykon_prev); hwSerial.print("  "); hwSerial.print(vykon);

			switch (smer) {
				case 1:
					if (vykon >= vykon_prev) {
						//hwSerial.print(" ^"); 
						strida += PWMDUTY_DELTA_PLUS; //pokud se vykon zvysuje pri smeru nahoru zvys stridu
						if (strida > maxStrida) {
							strida = maxStrida;
							smer = 2;
						}
					} else {
						smer = 2; //pokud je vykon nizsi zmen smer
						strida -= PWMDUTY_DELTA_MINUS;
						//hwSerial.print(" \\"); 
					}
					break;
				
				//smer dolu
				case 2:
					if (vykon >= vykon_prev && vykon > 0) {
						strida -= PWMDUTY_DELTA_MINUS; //pokud se vykon zvysuje pri smeru dolu sniz stridu
						//hwSerial.print(" \\"); 
						if (strida < 5) {
							strida = 5;
							smer = 1;
						}
					} else {
						smer = 1; //pokud je vykon nizsi zmen smer
						strida += PWMDUTY_DELTA_PLUS;
						//hwSerial.print(" ^"); 
					}
					break;
			}
			vykon_prev = vykon;
		}
	}

}

//==========================Test VA krivky =====================================
//==============================================================================
//nucene projede VA krivku FV z min. do max. stridy, pokud najde vyssi vykon
//nastavi stridu na tuto novou hodnotu
void testVA ()
{
	if (onVA)
	{
		// povolen test VA krivky
		currentMillis = millis();
		if ((unsigned long)(currentMillis - previousVA) >= (interval_VA * timeTestMPPT) && (strida > 50 && strida < 185) && (!rucne))
		{
			//kontrola VA krivky pokud ubehl nastaveny interval
			//neni ruc. rezim a strida je mezi 50 a 185
			int MAX_vykon = 0;
			int MAX_strida = 0;
			vykon = 0;
			lcd.clear();
			lcd.setCursor(0, 0);// sloupec, radek
			lcd.print("   Test MPPT    ");
			int m = 10;
			while ((m < maxStrida) && (vykon < vykonMax) )	{
				strida = m;
				set_PWM(strida);
				lcd.setCursor(0, 1);// sloupec, radek
				if ((int)(strida / 2.5) < 10) lcd.print(" ");
				lcd.print((int)(strida / 2.5));
				lcd.print("% ");
				//lcd.print(m);
				delay(10);
				mereni();
				if (vykon > MAX_vykon)	{
					MAX_vykon = vykon;
					MAX_strida = strida;
				}
				m += 5;
			}
			vykon = MAX_vykon;
			strida = MAX_strida;
			delay(500);
			previousVA = currentMillis;
		}

	}
}


//==============================Zobrazeni na LCD================================
//==============================================================================
void zobrazeni()
{
	//wdt_reset();
	unsigned long currentMillis = millis();
	if ((unsigned long)(currentMillis - previousMillis) >= interval_LCD) {

		lcd.setCursor(0, 0);// sloupec, radek
		if (napeti < 100 && napeti > 9) lcd.print(" ");
		if (napeti < 10) lcd.print("  ");
		lcd.print(napeti);
		lcd.print("V ");

		lcd.setCursor(5, 0);
		if ((digitalRead(enable_pin)) || (TeplBojl >= teplotaMax))
		{
			lcd.print("STOP ");
			if (digitalRead(enable_pin))lcd.print("E");
			if (TeplBojl >= teplotaMax)lcd.print ("T"); //
		}
		else
		{
			if (proud < 1000) lcd.print(" ");
			lcd.print(proud / 100);
			lcd.print(".");
			lcd.print((proud % 100) / 10);
			lcd.print("A ");
		}

		lcd.setCursor(11, 0);
		if (vykon < 1000 && vykon > 99) lcd.print(" ");
		if (vykon < 100 && vykon > 9) lcd.print("  ");
		if (vykon < 10) lcd.print("   ");
		lcd.print((int)vykon);
		lcd.print("W");

		if (rucne)
		{
			lcd.setCursor(0, 1);
			lcd.print("strida:");
			if ((int)(strida / 2.5) < 10) lcd.print(" ");
			lcd.print((int)(strida / 2.5));
			lcd.print("%      ");
			//odesle jas podsvetleni LCD na attinyx5
			Wire.beginTransmission(AttinyAddress); // zacatek komunikace
			Wire.write(10); // odesle hodnotu
			Wire.endTransmission();  // stop komunikace


		}
		else {
			lcd.setCursor(0, 1);
			if (vyroba < 10000000) lcd.print(" ");
			if (vyroba < 1000000) lcd.print(" ");
			if (vyroba < 100000) lcd.print(" ");
			if (vyroba < 10000) lcd.print(" ");
			lcd.print(vyroba / 1000);
			lcd.print(".");
			if (((vyroba % 1000) / 10) < 10) lcd.print("0");
			lcd.print((vyroba % 1000) / 10);
			lcd.print("kWh   ");

			lcd.setCursor(11, 1);
			if (TeplBojl < 100 && TeplBojl > 9) lcd.print(" ");
			if (TeplBojl < 10 && TeplBojl >= 0) lcd.print("  ");

			if (TeplBojl <= 0 || TeplBojl > 120)// kdyz je teplota mimo rozsah
			{
				lcd.setCursor(11, 1);
				lcd.print(" --");
			}
			else {
				lcd.print(TeplBojl);
			}
			lcd.write(1);//znacka stupne
			lcd.print("C ");

			//odesle jas podsvetleni LCD na attinyx5
			Wire.beginTransmission(AttinyAddress); // zacatek komunikace
			Wire.write(LCDbacklight);    // odesle hodnotu
			Wire.endTransmission();    // stop komunikace

		}

		previousMillis = currentMillis;
	}
}

//================ Nastaveni vyroby kWh pri zapnuti===================
//====================================================================
void nastaveni_kwh() {

	LCDbacklightLast = LCDbacklight; // uloz puvodni hodnotu podsviceni
	LCDbacklight = 10; // nastav podsviceni na max.
	set_kwh = true;
	byte Pkwh [7];
	int hodnota = 0;
	int pozice = 6;
	hodnotaL = vyroba;
	// rozdeli jednotliva cisla do poli
	Pkwh[0] = hodnotaL /  10000000;
	Pkwh[1] = (hodnotaL % 10000000) / 1000000;
	Pkwh[2] = (hodnotaL % 1000000) / 100000;
	Pkwh[3] = (hodnotaL % 100000) / 10000;
	Pkwh[4] = (hodnotaL % 10000) / 1000;
	Pkwh[6] = (hodnotaL % 1000) / 100;

	zobraz_kwh();
	delay(1000);

	do {
		zobraz_kwh();
		lcd.setCursor(pozice, 1);
		whichkey = PollKey();
		switch (whichkey) {

			case KeyDown: // zmena pozice nastavovaneho cisla
			pozice--;
			if (pozice == 5) pozice = 4; //preskoc desetinnou tecku
			if (pozice < 0) { //prejdi zase vpravo
				pozice = 6;
			}
			break;

			case KeyUp: //nastaveni cisla
			hodnota = Pkwh[pozice];
			hodnota++;
			if (hodnota > 9) hodnota = 0;
			Pkwh[pozice] = hodnota;
			hodnotaL = (((long)Pkwh[0] * 100000) + ((long)Pkwh[1] * 10000) + ((long)Pkwh[2] * 1000) + ((long)Pkwh[3] * 100) + ((long)Pkwh[4] * 10) + ((long)Pkwh[6]));
			hodnotaL *= 100; // uloz pole do long cisla

			break;

			case KeySel: // uloz nastavenou vyrobu
			vyroba = hodnotaL;
			lcd.noBlink();
			EEPROM_writeAnything(addr_vyroba, vyroba); //uloz do eeprom
			ulozeno(); break;

		}
	} while (showStatus);
	lcd.noBlink();
	set_kwh = false;
	LCDbacklight = LCDbacklightLast; // obnov puvodni hodnotu podsviceni
}
// ==================Zobrazeni celeho cisla kWh s nulami======================
void zobraz_kwh()
{
	lcd.clear();

	PrintLCD_P(14); // zobr. textu Suma vyroby
	lcd.setCursor(0, 1);
	if (hodnotaL < 10000000) lcd.print("0");
	if (hodnotaL < 1000000) lcd.print("0");
	if (hodnotaL < 100000) lcd.print("0");
	if (hodnotaL < 10000) lcd.print("0");
	lcd.print(hodnotaL / 1000);
	lcd.print(".");
	lcd.print((hodnotaL % 1000) / 100);
	lcd.print("kWh");
	lcd.blink();
}
//=============================================================================





//================================== LCD MENU==================================
//=============================================================================

void mainMenu() {
	menuTime = millis();
	LCDbacklightLast = LCDbacklight; // uloz puvodni hodnotu podsviceni
	do {
		delay(50);
		lcd.clear();
		LCDbacklight = 10; // nastav podsviceni na max.
		PrintLCD_P(12); // zobr. text *NASTAVENI*
		PrintLCDAt_P(offset , 0, 1);
		delay(200);
		whichkey = PollKey();
		switch (whichkey) {

			case KeyUp:
				if (offset >= 0) offset++;
				if (offset > 10) offset = 0;
				break;

			case KeyDown:
			switch (offset) {
			case 0: //exit z menu
				menu_exit();
				break;
			case 1: //nastaveni max. teploty
				menu_Mteplota();
				break;
			case 2: //nastaveni max. vykonu
				menu_Mvykon();
				break;
			case 3: //nastaveni periody testu VA krivky
				menu_perVA();
				break;
			case 4: //nastaveni ID menice
				menu_zmenaID();
				break;
			case 5: //kalibrace napeti
				menu_kalibraceV();
				break;
			case 6: //kalibrace proudu
				menu_kalibraceA();
				break;
			case 7: //nastaveni komunikace
				menu_komunikace();
				break;
			case 8: //podsviceni LCD
				menu_LCDbacklight();
				break;
			case 9: //max hodnoty
				menu_maxHodnoty();
				break;
			case 10: //max hodnoty
				menu_TovReset();
				break;
			}
			break;

		}
		lcd.clear();
	} while (showStatus);
	LCDbacklight = LCDbacklightLast; // obnov puvodni hodnotu podsviceni
}

//===================================================================
// nastaveni ID menice
void menu_zmenaID() {
	int hodnota = nodeID;
	do {
		lcd.clear();
		PrintLCDAt_P(offset, 0, 0);
		lcd.setCursor(1, 1);
		lcd.print(hodnota);
		whichkey = PollKey();
		switch (whichkey) {
			case KeyUp:
				if (hodnota >= 10) hodnota++;
				if (hodnota > 20) hodnota = 10;
				break;
			case KeyDown:
				nodeID = hodnota;
				//slave.setID(nodeID);// nastavi Modbus slave adresu
				EEPROM_writeAnything(12, nodeID); //uloz do eeprom
				ulozeno(); 
				break;
		}
	} while (showStatus);
}




//===================================================================
void menu_Mteplota() {
	int hodnota = teplotaMax;
	do {
		lcd.clear();
		PrintLCDAt_P(offset, 0, 0);
		lcd.setCursor(0, 1);
		lcd.print(hodnota);
		lcd.write(1);//znacka stupne
		lcd.print("C ");
		whichkey = PollKey();
		switch (whichkey) {
			case KeyUp:
			if (hodnota >= 40) hodnota += 1 ;
			if (hodnota > 90) hodnota = 40 ;
			break;
			case KeyDown:
			teplotaMax = hodnota;
			EEPROM_writeAnything(14, teplotaMax); //uloz do eeprom
			ulozeno(); break;
		}
	} while (showStatus);
}

//===================================================================
void menu_Mvykon() {
	int hodnota = vykonMax;
	do {
		lcd.clear();
		PrintLCDAt_P(offset, 0, 0);
		lcd.setCursor(0, 1);
		lcd.print(hodnota);
		lcd.print("W ");
		whichkey = PollKey();
		switch (whichkey) {
			case KeyUp:
			if (hodnota >= 1000) hodnota += 100 ;
			if (hodnota > 2800) hodnota = 1000 ;
			break;
			case KeyDown:
			vykonMax = hodnota;
			EEPROM_writeAnything(16, vykonMax); //uloz do eeprom
			ulozeno(); break;
		}
	} while (showStatus);
}

//===================================================================

void menu_maxHodnoty() {

	do {
		lcd.clear();
		PrintLCDAt_P(offset, 0, 0);
		lcd.setCursor(0, 1);
		lcd.print(maxV);
		lcd.print("V ");
		lcd.print(maxA / 100);
		lcd.print(".");
		lcd.print((maxA % 100) / 10);
		lcd.print("A ");
		lcd.print(maxW);
		lcd.print("W");

		whichkey = PollKey();
		switch (whichkey) {
		case KeyUp:
			break;
		case KeySel:
			maxV = 0;
			maxA = 0;
			maxW = 0;
			EEPROM_writeAnything(22, maxV);// int
			EEPROM_writeAnything(24, maxA);// int
			EEPROM_writeAnything(26, maxW);// int
			break;
		case KeyDown:
			showStatus = false;
			break;
		}

	} while (showStatus);
}


//===================================================================

void menu_perVA() {
	int hodnota = timeTestMPPT;
	do {
		lcd.clear();
		PrintLCDAt_P(offset, 0, 0);
		lcd.setCursor(0, 1);
		if (hodnota >= 0 && hodnota < 10) lcd.print(" "); //zapise mezeru pred jednomistne cislo
		lcd.print(hodnota);
		lcd.print(" min.");
		whichkey = PollKey();
		switch (whichkey) {
		case KeyUp:
			if (hodnota >= 0) hodnota += 5;//+=5
			if (hodnota > 60) hodnota = 0;//60
			break;
		case KeyDown:
			timeTestMPPT = hodnota;
			if (timeTestMPPT == 0) onVA = 0;
			else onVA = 1;
			EEPROM_writeAnything(20, onVA); //uloz do eeprom

			EEPROM_writeAnything(18, timeTestMPPT);//uloz do eeprom
			ulozeno(); 
			break;
		}
	} while (showStatus);
}

//===================================================================
// kalibrace napeti
void menu_kalibraceV() {
	int hodnota = kalibV; //
	do {
		lcd.clear();
		PrintLCDAt_P(offset, 0, 0);
		lcd.setCursor(1, 1);
		if (hodnota > 0) lcd.print("+");
		if (hodnota == 0) lcd.print(" ");
		if (hodnota == -5) lcd.print("-");
		lcd.print(hodnota / 10);
		lcd.print(".");
		lcd.print(abs(hodnota % 10));
		lcd.print("%  ");
		whichkey = PollKey();
		switch (whichkey) {

			case KeyUp:
			if (hodnota >= -50) hodnota += 5;
			if (hodnota > 50) hodnota = -50;
			break;
			case KeyDown:
			kalibV = hodnota;
			EEPROM_writeAnything(28, kalibV); //uloz do eeprom
			ulozeno(); break;
		}
	} while (showStatus);
}

//===================================================================
// kalibrace proudu
void menu_kalibraceA() {
	int hodnota = kalibA;
	do {
		lcd.clear();
		PrintLCDAt_P(offset, 0, 0);
		lcd.setCursor(1, 1);
		if (hodnota > 0) lcd.print("+");
		if (hodnota == 0) lcd.print(" ");
		if (hodnota == -5) lcd.print("-");
		lcd.print(hodnota / 10);
		lcd.print(".");
		lcd.print(abs(hodnota % 10));
		lcd.print("%  ");
		whichkey = PollKey();
		switch (whichkey) {

			case KeyUp:
			if (hodnota >= -50) hodnota += 5;
			if (hodnota > 50) hodnota = -50;
			break;
			case KeyDown:
			kalibA = hodnota;
			EEPROM_writeAnything(30, kalibA); //uloz do eeprom
			ulozeno(); break;
		}
	} while (showStatus);
}
//===================================================================
// nastaveni typu komunikace
void menu_komunikace() {
	int hodnota = KomTyp;
	do {
		lcd.clear();
		PrintLCDAt_P(offset, 0, 0);
		lcd.setCursor(0, 1);
		if (hodnota == 1)lcd.print("EasyTransfer");
		else lcd.print("Modbus RTU  ");
		whichkey = PollKey();
		switch (whichkey) {
			//case KeyDown:
			//  if (hodnota > 0) hodnota--; break;
			case KeyUp:
			if (hodnota >= 0) hodnota++;
			if (hodnota > 1) hodnota = 0;
			break;
			case KeyDown:
			KomTyp = hodnota;
			EEPROM_writeAnything(32, KomTyp); //uloz do eeprom
			ulozeno(); break;
		}
	} while (showStatus);


}
//===================================================================
void menu_TovReset() {
	int hodnota = 1;
	do {
		lcd.clear();
		PrintLCDAt_P(offset, 0, 0);
		lcd.setCursor(0, 1);
		if (hodnota == 1)lcd.print("NE        ");
		else lcd.print("ANO        ");
		whichkey = PollKey();
		switch (whichkey) {
			case KeyUp:
			if (hodnota >= 0) hodnota++;
			if (hodnota > 1) hodnota = 0;
			break;
			case KeyDown:
			showStatus = false;
			if (hodnota == 0) {
				FirstRun = 0x00000000; //nastav odlisnou hodnotu
				EEPROM_writeAnything(1020, FirstRun);
				wdt_enable(WDTO_1S); //zkraceni periody watchdogu
				delay(2000);// cekej na reset
			}
			break;

		}
	} while (showStatus);


}
//===================================================================
// nastaveni podsviceni
void menu_LCDbacklight() {

	EEPROM_readAnything(34, LCDbacklight);
	int hodnota = LCDbacklight;
	do {
		lcd.clear();
		PrintLCDAt_P(offset, 0, 0);
		lcd.setCursor(1, 1);
		lcd.print(hodnota);
		whichkey = PollKey();
		switch (whichkey) {
			case KeyUp:
			if (hodnota >= 0) hodnota++;
			if (hodnota > 10) hodnota = 0;
			LCDbacklight = hodnota;
			break;
			case KeyDown:
			LCDbacklight = hodnota;
			LCDbacklightLast = hodnota;
			EEPROM_writeAnything(34, LCDbacklight); //uloz do eeprom
			ulozeno(); break;
		}
	} while (showStatus);
}



void menu_exit() {

	showStatus = false;
}



//====================================================================
// vypis textu "ukladam..." na LCD

void ulozeno() {
	lcd.clear();
	PrintLCDAt_P(13, 0, 0);//zobrazi "ukladam..."
	delay(500);
	showStatus = false;
}

//===================================================================

void PrintLCDAt(char *inStr, char x, char y) {
	lcd.setCursor( x, y );
	delay(20);
	lcd.print(inStr);
	delay(40);
}
//===================================================================

void PrintLCDAt_P(int which, char x, char y) {
	lcd.setCursor( x, y );
	delay(20);
	PrintLCD_P(which);
}
//===================================================================

void PrintLCD_P(int which) {
	char buffer[21];
	strcpy_P(buffer, (char*)pgm_read_word(&(StringTable[which])));
	lcd.print(buffer);
	delay(40);
}
//===================================================================
//kontroluje analog. hodnotu stisknuteho tlacitka

char KeyScan() {
	int which, which2, diff, retVal;
	wdt_reset();
	which = analogRead(Buttons);
	//lcd.setCursor(0, 0);// DEBUG
	//lcd.print(which); // DEBUG
	delay(10);
	which2 = analogRead(Buttons);
	retVal = KeyInv;
	diff = abs(which - which2);
	if (diff < 12) {

		if (which > 710 && which < 800) retVal = KeySel; // obe tlacitka
		if (which > 600 && which < 700) retVal =  KeyUp; //tlac. nahoru
		if (which > 400 && which < 550) retVal =  KeyDown; //tlac. dolu
		if (which > 210 && which < 300) retVal =  KeyExit; // nepouzito
	}
	return retVal;
}

//==================================================================
//pri pohybu v Menu ceka na platnou hodnotu tlacitka,
//

char PollKey() {

	char Whichkey;
	do {
		Whichkey = KeyScan();
		delay(60);
		//cinosti vykonavane behem zobrazeni menu
		if ((digitalRead(enable_pin)) || (TeplBojl >= teplotaMax) || (set_kwh)) strida = 0; // provoz menice neni povolen, nastav stridu na 0
		else {
			mereni();
			rizeni();
		}
		set_PWM(strida);
		//odesle jas podsvetleni LCD na attinyx5
		Wire.beginTransmission(AttinyAddress); // zacatek komunikace
		Wire.write(LCDbacklight);    // odesle hodnotu
		Wire.endTransmission();  // stop komunikace


		//pri necinnosti v menu zpet na hlavni obrazovku
		if (millis() > (menuTime + menuTimeExit)) {
			showStatus = false;
		}

	} while ((Whichkey == KeyInv) && showStatus);
	menuTime = millis();
	delay(80);
	return Whichkey;
}

