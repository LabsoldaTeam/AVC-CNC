/*
 Name:		AVC_CNC.ino
 Created:	4/13/2021 7:38:23 PM
 Author:	Felippe Kalil Mendon�a (Atualiza��o 2021) e Alex Sandro Pereira (Original - v1.0)
v2.0 - Add Modo Auto Ref, update serial, e add novos sons (inc, dec,..)
v2.1 - Save data eeprom
*/

/*Firmware para compensa��o da posi��o
 * Autor: Alex Sandro Pereira (Original - v1.0)
 * Institudo de Mecatr�nica LABSOLDA
 * Descri��o: Este software tem como objetivo receber o valor de
 * tens�o da fonte de soldagem e compensar a posi��o da tocha.
 *
 * Ardu�no conex�es:
 * pin 02 - ON/OFF
 * pin 03 - Buzzer
 * pin 08 - Compensa��o -
 * pin 09 - Compensa��o +
 * pin 14 - TX
 * pin 15 - RX
 *
 *
 */


#define PLOT_SERIAL
 //#define DEBUG_SERIAL
//#define TROCA_SERIAL

//         BIBLIOTECAS
#include <LiquidCrystal.h> 
#include "EstruturasExtras.h"


//     CONFIGURA��O DE PINOS
#define ON_OFF_PIN           2 
#define BUZZER_PIN           3 
#define MENOS_PIN            47
#define MAIS_PIN             49
#define TX_PIN               16
#define RX_PIN               17
#define LEDVERMELHO_PIN      37
#define LEDAMARELO_PIN       39
#define LEDVERDE_PIN         41

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


//     DECLARA��O DAS VARI�VEIS GLOBAIS
bool ligado;						//sinal correspondente a chave on off
bool  lerTensao;                    //dispara uma nova leitura da tens�o pela interrup��o
float tensao_da_fonte;              //valor da tens�o 
byte   byte_recebido_parte1;        //os valores recebidos s�o divididos em dois
byte   byte_recebido_parte2;        //a segunda parte s�o os decimais
int   dados_recebidos_da_fonte[5];  //recebe os 4 bytes da fonte
byte  compensacao;                  //informa se a compensa��o esta ativa ou n�o
byte  faixa_tensao_lida;            //alerta que a tens�o da fonte esta fora da faixa informada como min e max
int   base_tempo1_buzzer;           //base de tempo para o buzzer
int   base_tempo2_buzzer;           //2� base de tempo para o buzzer
int   base_tempo1_led;              //base de tempo para o led
int   base_tempo2_led;              //2� base de tempo para o led
int   byte_da_vez;                  //para a l�gica de recebimento dos bytes da fonte na fun��o de leitura da tens�o


EstadoTensao estadoTensaoOff = Off;      // Estado da l�gica de arco aberto
EstadoTensao estadoAutoRef = Off;      // Estado da l�gica de Auto refer�ncia
long tAbriu = 0;						// Momento em que o arco foi aberto
StatusBuzzer alarmeAtual = SemAlarme;
StatusLeitura  estadoComunicacao = Erro;           //se 1 a comunicacao esta ok, se 0 esta com erro.


//    DEFINI��ES DE PAR�METROS 
#define INICIA_COMUNCACAO          99
#define TENSAO_ID                  49
#define CORRENTE_ID                48
#define PARAM_1_SEGUNDO           0xC2F7      // PARA 1 SEGUNDO = 65536-(16MHz/1024/1Hz) = 49911 = 0xC2F7
#define PARAM_0_1_SEGUNDO         0xF9E6      //0x137F      // PARA 0,1 SEGUNDO = 4991 = 0x137F
#define ALPHA_FILTRO_TENSAO		0.5 // Alpha empregado na filtragem do sinal te tens�o
#define T_LOOP_ATUACAO			300 // ms
#define DURACAO_MAX_CORRECAO	100 // ms


//     DECLAR��O DAS FUN��ES
void leituras(void);
bool estadoArcoPrincipal();
bool atualizaStatusReferencia();
void parar();
void descer();
void subir();
void acionamentos(bool atua);
void ihm(void);
void alarmeBuzzer();
void alertaLed(StatusLeitura statusAlerta);
int checaBotao();
bool apagaPisca();
void menuTensaoRef(bool apaga);
void menuAutoRef(bool apaga);
void shieldLcd(void);
StatusLeitura verificaDadosDaLeitura(float valorLido);


//      OUTROS
//SoftwareSerial Serial(rxPin, txPin); //configurar comunica��o


void setup()
{
	//configurar os pinos
	pinMode(ON_OFF_PIN, INPUT_PULLUP);
	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(MENOS_PIN, OUTPUT);
	pinMode(MAIS_PIN, OUTPUT);
	pinMode(LEDVERMELHO_PIN, OUTPUT);
	pinMode(LEDAMARELO_PIN, OUTPUT);
	pinMode(LEDVERDE_PIN, OUTPUT);
	digitalWrite(BUZZER_PIN, LOW);
	digitalWrite(MENOS_PIN, LOW);
	digitalWrite(MAIS_PIN, LOW);
	digitalWrite(LEDVERMELHO_PIN, LOW);
	digitalWrite(LEDAMARELO_PIN, LOW);
	digitalWrite(LEDVERDE_PIN, LOW);


	//configurar comunica��o
#ifdef TROCA_SERIAL
	Serial.begin(57600, SERIAL_8E1);
	Serial2.begin(115200);
#else
	Serial2.begin(57600, SERIAL_8E1);
	Serial.begin(115200);
#endif


	//Configurar LCD
	lcd.begin(16, 2);                    //SETA A QUANTIDADE DE COLUNAS(16) E O N�MERO DE LINHAS(2) DO DISPLAY. EM SUMA: UMA MATRIZ DE 16 COLUNAS E 2 LINHAS
	lcd.setCursor(0, 0);                 //SETA A POSI��O EM QUE O CURSOR INCIALIZA(LINHA 1) 


   // Configura��o do timer1 
	TCCR1A = 0;                         //confira timer para opera��o normal pinos OC1A e OC1B desconectados
	TCCR1B = 0;                         //limpa registrador
	TCCR1B |= (1 << CS10) | (1 << CS12);    // configura prescaler para 1024: CS12 = 1 e CS10 = 1
	TCNT1 = PARAM_0_1_SEGUNDO;          // incia timer com valor para que estouro ocorra em 0,1 segundo
	TIMSK1 |= (1 << TOIE1);             // habilita a interrup��o do TIMER1
	loadParametros();
}

void loop()
{
	leituras();     //ler
	acionamentos(atualizaStatusReferencia()); //agir
	ihm();          //comunicar
	saveParametros(); //verifica altera��o de par�metros a cada 3s, e se for o caso salva par�metros atualizados
}


/*
 *         LEITURAS
 *
 * Fun��es respons�veis por ler grandezas f�sicas
 * devem estar aqui;
 *
 */
void leituras(void) {
	ligado = !digitalRead(ON_OFF_PIN); // L� a chave ON OFF

	static int tentativasSemSucesso;
	static auto lendo = false;
	if (lerTensao) {		//ler a cada chamada da interrup��o com tempo configurado    		
		lerTensao = false;                                                                    //aguarda o tempo para a pr�xima leitura
		if (tentativasSemSucesso++ >= 10) {
			tentativasSemSucesso = 0;
			estadoComunicacao = Erro;
			char resetCom[] = { 0xFF,0xFF,0xFF,0xFF,NULL };
#ifdef TROCA_SERIAL
			Serial.write(resetCom);
#ifdef DEBUG_SERIAL
			Serial2.println("erro - Reset");
#endif
#else
			Serial2.write(resetCom);
#ifdef DEBUG_SERIAL
			Serial.println("erro - Reset");
#endif
#endif

			lendo = false;
			byte_da_vez = 0;                                                                  //prepara a vari�vel para a l�gica seguinte
		}
		if (!lendo)
		{
#ifdef TROCA_SERIAL
			Serial.write(INICIA_COMUNCACAO);                                                  //inicia a comunica��o
			Serial.write(TENSAO_ID);
#else
			Serial2.write(INICIA_COMUNCACAO);                                                  //inicia a comunica��o
			Serial2.write(TENSAO_ID);
#endif
			lendo = true;
			byte_da_vez = 0;                                                                  //prepara a vari�vel para a l�gica seguinte
		}
	}

#ifdef TROCA_SERIAL
	const auto available = Serial.available();
#else
	const auto available = Serial2.available();
#endif

	if (available > 0) {									//se receber algo na serial
		tentativasSemSucesso = 0;									//zera alerta de falta de comunica��o

#ifdef TROCA_SERIAL
		if (byte_da_vez < 5)
			dados_recebidos_da_fonte[byte_da_vez] = Serial.read();		//armazena os 4 bytes informados
#ifdef DEBUG_SERIAL
		Serial2.print("Dado ");
		Serial2.print(byte_da_vez);
		Serial2.print(": ");
		for (auto i = 0; i < 5; i++)
		{
			Serial2.print(dados_recebidos_da_fonte[i]);
			if (i == 4)
				Serial2.println(" ");
			else
				Serial2.print(" ");
		}
#endif
#else
		if (byte_da_vez < 5)
			dados_recebidos_da_fonte[byte_da_vez] = Serial2.read();		//armazena os 4 bytes informados
#ifdef DEBUG_SERIAL
		Serial.print("Dado ");
		Serial.print(byte_da_vez);
		Serial.print(": ");
		for (auto i = 0; i < 5; i++)
		{
			Serial.print(dados_recebidos_da_fonte[i]);
			if (i == 4)
				Serial.println(" ");
			else
				Serial.print(" ");
		}
#endif
#endif
		switch (byte_da_vez)
		{
		case 0:
			if (dados_recebidos_da_fonte[0] != INICIA_COMUNCACAO)
				byte_da_vez = 0;
			else
				byte_da_vez++;
			break;
		case 1:
			if (dados_recebidos_da_fonte[1] != TENSAO_ID)
				byte_da_vez = 0;
			else
				byte_da_vez++;
			break;
		case 2: //se for o 3� byte
			byte_recebido_parte1 = dados_recebidos_da_fonte[byte_da_vez];	//salva o 3� byte
			byte_da_vez++;
			break;
		case 3: //se for o 4� byte
			byte_recebido_parte2 = dados_recebidos_da_fonte[byte_da_vez];	//salva o 4� byte
			byte_da_vez++;
			break;
		case 4: //se for o 5� byte ('O')
			if (dados_recebidos_da_fonte[byte_da_vez] == 'O')
			{
				const short temp = byte_recebido_parte1 | byte_recebido_parte2 << 8;
				auto tensaoTemp = static_cast<float>(temp);// ((byte_recebido_parte1)+(byte_recebido_parte2 * 256));                 //junta o 3� e 4� byte, como 100 e 10 = 100,10
				tensaoTemp /= 10.0;                                                 //aqui tem o valor recebido de tens�o da fonte
				estadoComunicacao = verificaDadosDaLeitura(tensaoTemp) ? Ok : Erro;   //retorna 1(ok), caso a leitura esteja dentro da faixa pr�-estabelecida
				if (estadoComunicacao == Ok)
					tensao_da_fonte = tensaoTemp * ALPHA_FILTRO_TENSAO + tensao_da_fonte * (1 - ALPHA_FILTRO_TENSAO);
				else
					tentativasSemSucesso = 10;
#ifdef PLOT_SERIAL
#ifdef TROCA_SERIAL
				Serial2.print("SetPoint:");
				Serial2.print(paramsAvc.setpoint);
				Serial2.print(" Tensao:");
				Serial2.println(tensao_da_fonte);
#else
				Serial.print("SetPoint:");
				Serial.print(paramsAvc.setpoint);
				Serial.print(" Tensao:");
				Serial.println(tensao_da_fonte);
#endif
#endif
#ifdef DEBUG_SERIAL
#ifdef TROCA_SERIAL
				Serial2.print("Atualizado -> ");
				Serial2.println(tensao_da_fonte);
#else
				Serial.print("Atualizado -> ");
				Serial.println(tensao_da_fonte);
#endif
#endif
			}
			else
			{
#ifdef DEBUG_SERIAL
#ifdef TROCA_SERIAL
				Serial2.println("Dado Nok");
#else
				Serial.println("Dado Nok");
#endif
#endif
				tentativasSemSucesso = 10;
			}
			memset(dados_recebidos_da_fonte, 0, 5);
			lendo = false;
			byte_da_vez = 0;
			break;
		default:;
		}
	}

}


/*
 *         STATUS TENS�O
 *
 * Fun��es respons�veis pela atualiza��o do estado da tens�o,
 * e da refer�ncia para o modo AutoRef;
 *
 */

bool estadoArcoPrincipal()
{
	if (estadoComunicacao == Erro)
		return false;

	if (tensao_da_fonte < PARAM_TENSAO_MINIMA_OFF)
		estadoTensaoOff = Off;

	switch (estadoTensaoOff)
	{
	case Off:
		if (tensao_da_fonte > PARAM_TENSAO_MINIMA_OFF + 2.0)
		{
			estadoTensaoOff = Estabilizacao;
			tAbriu = millis();
		}
		break;
	case Estabilizacao:
		if (millis() > tAbriu + PARAM_TEMPO_OFF * 1000)
			estadoTensaoOff = Soldagem;
		break;
	case Soldagem:
		return true;
	default:;
	}
	return false;
}

bool atualizaStatusReferencia()
{
	if (!estadoArcoPrincipal() || !ligado)
		return false;

	if (paramsAvc.modoOperacao == TensaoRef)
		return true;

	if (tensao_da_fonte < 5)
		estadoAutoRef = Off;

	switch (estadoAutoRef)
	{
	case Off:
		if (tensao_da_fonte > 20)
		{
			estadoAutoRef = Estabilizacao;
			tAbriu = millis();
		}
		break;
	case Estabilizacao:
		if (millis() > tAbriu + paramsAvc.tempoEstab * 1000)
		{
			estadoAutoRef = Soldagem;
			paramsAvc.setpoint = tensao_da_fonte;
		}
		break;
	case Soldagem:
		return true;
	default:;
	}
	return false;
}


/*
 *         ACIONAMENTO
 *
 * Fun��es respons�veis pelas rotinas de controle
 * devem estar aqui;
 *
 */

void parar()
{
	digitalWrite(MENOS_PIN, LOW);
	digitalWrite(MAIS_PIN, LOW);
}

void descer()
{
	digitalWrite(MENOS_PIN, HIGH);
	digitalWrite(MAIS_PIN, LOW);
}

void subir()
{
	digitalWrite(MENOS_PIN, LOW);
	digitalWrite(MAIS_PIN, HIGH);
}


void acionamentos(bool atua) {                                   //a compensa��o � ativa a cada leitura correta da tens�o
	const auto tAtual = millis();
	bool cicloCorr = (tAtual % T_LOOP_ATUACAO) == 0;
	static long tAtuacao = 0;
	if (tAtual - tAtuacao < DURACAO_MAX_CORRECAO)
		atua = false;
	if (atua)
	{
		if (tensao_da_fonte > (paramsAvc.setpoint + paramsAvc.histerese))                  //ent�o verifica se precisa compensar
		{
			if (cicloCorr)
			{
				descer();
				tAtuacao = tAtual;
			}
		}
		else if (tensao_da_fonte < (paramsAvc.setpoint - paramsAvc.histerese))
		{
			if (cicloCorr)
			{
				subir();
				tAtuacao = tAtual;
			}
		}
		else
			parar();
	}
	else
		parar();
}



/*
 *         INTERFACE HUMANO M�QUINA (IHM)
 *
 * Qualquer fun��o respons�vel por se comunicar com o
 * operador deve ser colocado aqui
 *
 *
 */
void ihm() {
	static auto estadoAnt = -1;
	if (estadoComunicacao != estadoAnt)
		alarmeAtual = estadoComunicacao == Ok ? SemAlarme : Alarme2;
	estadoAnt = estadoComunicacao;
	alarmeBuzzer();
	alertaLed(estadoComunicacao);
	shieldLcd();
}



/*
 *                ALERTA BUZZER
 *
 * Avisa com o sinal do buzzer se a conex�o com a serial da fonte
 * foi bem sucedida ou n�o.
 *
 * Tipos de alertas
 * 1 = ok
 * 2 = faixa_tensao_lida = erro
 * 3 = conex�o com a serial com problema
 *
 */
int alerta_sem_erro;
float seno;
int frequencia;
int buzzer_ja_esta_ligado;
int temporaria_1_alertaBuzzer;
int temporaria_2_alertaBuzzer;

void alarmeBuzzer() {
	//return;
	switch (alarmeAtual) {
	case SemAlarme:
		noTone(BUZZER_PIN);
		alerta_sem_erro = 1;
		temporaria_1_alertaBuzzer = 0;
		break;

	case Inc:
		if (temporaria_1_alertaBuzzer++ == 1)
			tone(BUZZER_PIN, 1131.2);
		else if (temporaria_1_alertaBuzzer >= 3)
			alarmeAtual = SemAlarme;
		break;

	case Dec:
		if (temporaria_1_alertaBuzzer++ == 1)
			tone(BUZZER_PIN, 30.8);
		else if (temporaria_1_alertaBuzzer >= 3)
			alarmeAtual = SemAlarme;
		break;

	case Enter:
		if (temporaria_1_alertaBuzzer++ == 1)
			tone(BUZZER_PIN, 416.6);
		else if (temporaria_1_alertaBuzzer == 5)
			noTone(BUZZER_PIN);
		else if (temporaria_1_alertaBuzzer == 10)
			tone(BUZZER_PIN, 1131.2);
		else if (temporaria_1_alertaBuzzer > 20)
			alarmeAtual = SemAlarme;
		break;

	case Volta:
		if (temporaria_1_alertaBuzzer++ == 1)
			tone(BUZZER_PIN, 1131.2);
		else if (temporaria_1_alertaBuzzer == 10)
			noTone(BUZZER_PIN);
		else if (temporaria_1_alertaBuzzer == 15)
			tone(BUZZER_PIN, 416.6);
		else if (temporaria_1_alertaBuzzer > 20)
			alarmeAtual = SemAlarme;
		break;

	case Alarme1:

		break;

	case Alarme2:
		// temporaria_2_alertaBuzzer = 0;
		if (temporaria_1_alertaBuzzer++ == 1)
			tone(BUZZER_PIN, 30.8);
		else if (temporaria_1_alertaBuzzer == 9)
			noTone(BUZZER_PIN);
		else if (temporaria_1_alertaBuzzer == 15)
			tone(BUZZER_PIN, 1131.2);//392
		else if (temporaria_1_alertaBuzzer == 45)
			noTone(BUZZER_PIN);
		else if (temporaria_1_alertaBuzzer > 45)
			alarmeAtual = SemAlarme;
		delay(1);
		break;
	default:;
	}
}

/*
 *                ALERTA LED
 *
 *
 */
#define LED_VERDE_LIGADO          digitalWrite(LEDVERDE_PIN,HIGH)
#define LED_AMARELO_LIGADO        digitalWrite(LEDAMARELO_PIN,HIGH)
#define LED_VERMELHO_LIGADO       digitalWrite(LEDVERMELHO_PIN,HIGH)
#define LED_VERDE_DESLIGADO       digitalWrite(LEDVERDE_PIN,LOW)
#define LED_AMARELO_DESLIGADO     digitalWrite(LEDAMARELO_PIN,LOW)
#define LED_VERMELHO_DESLIGADO    digitalWrite(LEDVERMELHO_PIN,LOW)

void alertaLed(const StatusLeitura statusAlerta) {
	switch (statusAlerta) {
	case Ok:
		LED_VERDE_LIGADO;
		LED_AMARELO_DESLIGADO;
		LED_VERMELHO_DESLIGADO;
		break;

	case Erro:
		if (base_tempo1_led == 0) {
			base_tempo1_led = 10;
		}
		else if (base_tempo1_led > 5) {
			LED_VERDE_DESLIGADO;
			LED_AMARELO_LIGADO;
			LED_VERMELHO_DESLIGADO;
		}
		else {
			LED_VERDE_DESLIGADO;
			LED_AMARELO_DESLIGADO;
			LED_VERMELHO_DESLIGADO;
		}
		break;
	default:
		break;
	}
}


/*
 *                shieldLCD
 * Controla o LCD 16x2 e os bot�es que est�o na mesma placa
 *
 */
#define CHAVE_ON_OFF  -1
#define BOTAO_DIREITO  1
#define BOTAO_CIMA     2
#define BOTAO_BAIXO    3
#define BOTAO_ESQUERDA 4
#define BOTAO_SELECT   5
#define BOTAO_NENHUM   6
#define SOLTO         1
#define PRESSIONADO   0
#define PARAM_PV        1
#define PARAM_SP        2
#define PARAM_HI        3   
#define NUMERO_DE_PARAM  3  
int temporaria_1_shieldLCD;
int temporaria_2_shieldLCD;
int temporaria_3_shieldLCD;
int temporaria_4_shieldLCD;
int temporaria_5_shieldLCD;
int _parametro_da_vez;
int _botao_select_antes = 0;
int _botao_select_agora = 0;
int tempo_botao_pressionado = 15;

int checaBotao() // Identifica o bot�o pressionado
{
	auto botaoPressionado = 0;

	if ((analogRead(0)) < 80) { //SE A LEITURA DO PINO FOR MENOR QUE 80, FAZ 
		if (temporaria_1_shieldLCD++ >= tempo_botao_pressionado) {
			temporaria_1_shieldLCD = 0;
			botaoPressionado = BOTAO_DIREITO;
			/*   if (tempo_botao_pressionado-- < 1) {
				   tempo_botao_pressionado = 0;
			   }*/
		}
		temporaria_2_shieldLCD = 0;
		temporaria_3_shieldLCD = 0;
		temporaria_4_shieldLCD = 0;
		temporaria_5_shieldLCD = 0;
	}
	else if ((analogRead(0)) < 200) { //SE A LEITURA DO PINO FOR MENOR QUE 200, FAZ
		if (temporaria_2_shieldLCD++ >= tempo_botao_pressionado) {
			temporaria_2_shieldLCD = 0;
			botaoPressionado = BOTAO_CIMA;
			if (tempo_botao_pressionado-- < 1)
				tempo_botao_pressionado = 0;
		}
		temporaria_1_shieldLCD = 0;
		temporaria_3_shieldLCD = 0;
		temporaria_4_shieldLCD = 0;
		temporaria_5_shieldLCD = 0;
	}
	else if ((analogRead(0)) < 400) { //SE A LEITURA DO PINO FOR MENOR QUE 400, FAZ  
		if (temporaria_3_shieldLCD++ >= tempo_botao_pressionado) {
			temporaria_3_shieldLCD = 0;
			botaoPressionado = BOTAO_BAIXO;
			if (tempo_botao_pressionado-- < 1)
				tempo_botao_pressionado = 0;
		}
		temporaria_1_shieldLCD = 0;
		temporaria_2_shieldLCD = 0;
		temporaria_4_shieldLCD = 0;
		temporaria_5_shieldLCD = 0;
	}
	else if ((analogRead(0)) < 600) { //SE A LEITURA DO PINO FOR MENOR QUE 600, FAZ  
		if (temporaria_4_shieldLCD++ >= tempo_botao_pressionado) {
			temporaria_4_shieldLCD = 0;
			botaoPressionado = BOTAO_ESQUERDA;

			/*   if (tempo_botao_pressionado-- < 1) {
				   tempo_botao_pressionado = 0;
			   }*/
		}
		temporaria_1_shieldLCD = 0;
		temporaria_2_shieldLCD = 0;
		temporaria_3_shieldLCD = 0;
		temporaria_5_shieldLCD = 0;
	}
	else if ((analogRead(0)) < 800) { //SE A LEITURA DO PINO FOR MENOR QUE 800, FAZ 
		if (temporaria_5_shieldLCD++ >= tempo_botao_pressionado) {
			temporaria_5_shieldLCD = 0;
			botaoPressionado = BOTAO_SELECT;
		}
		temporaria_1_shieldLCD = 0;
		temporaria_2_shieldLCD = 0;
		temporaria_3_shieldLCD = 0;
		temporaria_4_shieldLCD = 0;
	}
	else {
		botaoPressionado = BOTAO_NENHUM;
		_botao_select_antes = SOLTO;
		tempo_botao_pressionado = 15;
	}
	return botaoPressionado;
}

bool apagaPisca()
{
	static auto piscaTemp = 0;
	if (piscaTemp++ < 40)
		return false;
	if (piscaTemp++ < 80)
		return true;
	if (piscaTemp++ > 81)
		piscaTemp = 0;
	return true;
}

void menuDesligado()
{
	lcd.print("   DESLIGADO    ");
	lcd.setCursor(0, 1);
	lcd.print("  ARCO: ");
	lcd.print(tensao_da_fonte, 1);
	lcd.print(" V   ");
}

void menuTensaoRef(const bool apaga)
{
	if (_parametro_da_vez == PARAM_PV && apaga)
		lcd.print("    ");
	else
		lcd.print("(A) ");
	lcd.print("ARCO: ");
	lcd.print(tensao_da_fonte, 1);
	lcd.print(" V");
	lcd.setCursor(0, 1);
	lcd.print("Ref:");
	if (_parametro_da_vez == PARAM_SP && apaga)
		lcd.print("     ");
	else
		lcd.print(paramsAvc.setpoint, 1);
	if (paramsAvc.setpoint >= 10)
		lcd.setCursor(8, 1);
	else
		lcd.setCursor(7, 1);
	lcd.print("V ");
	lcd.setCursor(10, 1);
	lcd.print("+-");
	if (_parametro_da_vez == PARAM_HI && apaga)
		lcd.print("   ");
	else
		lcd.print(paramsAvc.histerese, 1);
	lcd.print("V");
}

void menuAutoRef(const bool apaga)
{
	if (_parametro_da_vez == PARAM_PV && apaga)
		lcd.print("    ");
	else
		lcd.print("(B) ");
	lcd.print("ARCO: ");
	lcd.print(tensao_da_fonte, 1);
	lcd.print(" V");
	lcd.setCursor(0, 1);
	lcd.print("tEst:");
	if (_parametro_da_vez == PARAM_SP && apaga)
		lcd.print("    ");
	else
		lcd.print(paramsAvc.tempoEstab, 1);
	if (paramsAvc.tempoEstab >= 10)
		lcd.setCursor(9, 1);
	else
		lcd.setCursor(8, 1);
	lcd.print("s ");
	lcd.setCursor(10, 1);
	lcd.print("+-");
	if (_parametro_da_vez == PARAM_HI && apaga)
		lcd.print("   ");
	else
		lcd.print(paramsAvc.histerese, 1);
	lcd.print("V");
}

void menuManual(const bool apaga)
{
	if (_parametro_da_vez == PARAM_PV && apaga)
		lcd.print("    ");
	else
		lcd.print("(C) ");
	lcd.print("ARCO: ");
	lcd.print(tensao_da_fonte, 1);
	lcd.print(" V");
	lcd.setCursor(0, 1);
	lcd.print("    MANUAL      ");
}

void shieldLcd() {
	if (ligado)
		switch (checaBotao()) {
		case BOTAO_SELECT:
		case BOTAO_DIREITO:
			alarmeAtual = Enter;
			if (_botao_select_antes == SOLTO) {
				if (_parametro_da_vez++ >= PARAM_HI)
					_parametro_da_vez = 0;
				_botao_select_antes = PRESSIONADO;
			}
			break;
		case BOTAO_ESQUERDA:
			alarmeAtual = Volta;
			if (_botao_select_antes == SOLTO) {
				if (_parametro_da_vez-- <= 0)
					_parametro_da_vez = PARAM_HI;
				_botao_select_antes = PRESSIONADO;
			}
			break;
		case BOTAO_CIMA:
			if (!ligado)
				subir();
			else
				switch (_parametro_da_vez)
				{
				case PARAM_PV:
					alarmeAtual = Inc;
					paramsAvc.modoOperacao = paramsAvc.modoOperacao ? TensaoRef : AutoRef;
					break;
				case PARAM_SP:
					alarmeAtual = Inc;
					if (paramsAvc.modoOperacao == TensaoRef)
					{
						if (paramsAvc.setpoint < PARAM_VALOR_SP_MAXIMO_TENSAO)
							paramsAvc.setpoint = paramsAvc.setpoint + 0.1;
					}
					else
						if (paramsAvc.tempoEstab < PARAM_VALOR_ESTAB_MAXIMO)
							paramsAvc.tempoEstab = paramsAvc.tempoEstab + 0.1;

					break;
				case PARAM_HI:
					alarmeAtual = Inc;
					if (paramsAvc.histerese < PARAM_VALOR_HI_MAXIMO_TENSAO)
						paramsAvc.histerese = paramsAvc.histerese + 0.1;
					break;
				default:
					break;
				}
			break;
		case BOTAO_BAIXO:
			switch (_parametro_da_vez)
			{
			case PARAM_PV:
				alarmeAtual = Dec;
				paramsAvc.modoOperacao = paramsAvc.modoOperacao ? TensaoRef : AutoRef;
				break;
			case PARAM_SP:
				alarmeAtual = Dec;
				if (paramsAvc.modoOperacao == TensaoRef)
				{
					if (paramsAvc.setpoint > PARAM_VALOR_SP_MINIMO_TENSAO)
						paramsAvc.setpoint = paramsAvc.setpoint - 0.1;
				}
				else
					if (paramsAvc.tempoEstab > PARAM_VALOR_ESTAB_MINIMO)
						paramsAvc.tempoEstab = paramsAvc.tempoEstab - 0.1;
				break;
			case PARAM_HI:
				alarmeAtual = Dec;
				if (paramsAvc.histerese > PARAM_VALOR_HI_MINIMO_TENSAO)
					paramsAvc.histerese = paramsAvc.histerese - 0.1;
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
	else
		switch (checaBotao()) {
		case BOTAO_CIMA:
			subir();
			break;
		case BOTAO_BAIXO:
			descer();
			break;
		default:
			parar();
			break;
		}

	const auto apaga = apagaPisca();

	lcd.setCursor(0, 0);

	if (!ligado)
		menuDesligado();
	else
		switch (paramsAvc.modoOperacao)
		{
		default:
		case TensaoRef:
			menuTensaoRef(apaga);
			break;
		case AutoRef:
			menuAutoRef(apaga);
			break;
		}
}


/*
 *         VERIFICA DADOS DA LEITURA
 *
 * Verifica se o dados que leu de tens�o da fonte est�o dentro da
 * faixa pr�viamente configurada.
 */
StatusLeitura verificaDadosDaLeitura(const float valorLido) {
	if (valorLido >= PARAM_VALOR_MINIMO_TENSAO && valorLido <= PARAM_VALOR_MAXIMO_TENSAO)
		return Ok;
	return Erro;
}


/*
 *         INTERRUP��O
 *
 * Interrup��o a cada 0,1 segundo para quaisquer base de tempo que o
 * software precise.
 *
 */

ISR(TIMER1_OVF_vect)
{
	lerTensao = true;          //Inicia uma nova leitura da tens�o da fonte	

	if (base_tempo1_buzzer)
		base_tempo1_buzzer--;

	if (base_tempo2_buzzer)
		base_tempo2_buzzer--;

	if (base_tempo1_led)
		base_tempo1_led--;

	if (base_tempo2_led)
		base_tempo2_led--;

	TCNT1 = PARAM_0_1_SEGUNDO;   // Renicia TIMER
}
