/*
 Name:		AVC_CNC.ino
 Created:	4/13/2021 7:38:23 PM
 Author:	Kalil (Atualização 2021)
v2.0 - Add Modo Auto Ref, e add novos sons (inc, dec,..)
TODO v2.1 - Save data eeprom
*/

/*Firmware para compensação da posição
 * Autor: Alex Sandro Pereira (Original - v1.0)
 * Institudo de Mecatrônica LABSOLDA
 * Descrição: Este software tem como objetivo receber o valor de
 * tensão da fonte de soldagem e compensar a posição da tocha.
 *
 * Arduíno conecções:
 * pin 03 - Buzzer
 * pin 08 - Compensação -
 * pin 09 - Compensação +
 * pin 14 - TX
 * pin 15 - RX
 *
 *
 */
 //         BIBLIOTECAS
#include <LiquidCrystal.h> 


//     CONFIGURAÇÃO DE PINOS
#define BUZZER_PIN           3 
#define MENOS_PIN            47
#define MAIS_PIN             49
#define TX_PIN               16
#define RX_PIN               17
#define LEDVERMELHO_PIN      37
#define LEDAMARELO_PIN       39
#define LEDVERDE_PIN         41

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


//     DECLARAÇÃO DAS VARIÁVEIS GLOBAIS
byte  lerTensao;                    //dispara uma nova leitura da tensão pela interrupção
float tensao_da_fonte;              //valor da tensão 
byte   byte_recebido_parte1;        //os valores recebidos são divididos em dois
byte   byte_recebido_parte2;        //a segunda parte são os decimais
int   dados_recebidos_da_fonte[5];  //recebe os 4 bytes da fonte
byte  compensacao;                  //informa se a compensação esta ativa ou não
byte  faixa_tensao_lida;            //alerta que a tensão da fonte esta fora da faixa informada como min e max
int   base_tempo1_buzzer;           //base de tempo para o buzzer
int   base_tempo2_buzzer;           //2ª base de tempo para o buzzer
int   base_tempo1_led;              //base de tempo para o led
int   base_tempo2_led;              //2ª base de tempo para o led
int   byte_da_vez;                  //para a lógica de recebimento dos bytes da fonte na função de leitura da tensão
float param_Histerese = 0.5;        // Valor da histerese em V
float param_Setpoint = 12;			// Ref em V
float paramTempoEstab = 3.5;			// Tempo de estabilização em s, para atualização automática da referência
enum ModoOperacao
{
	TensaoRef = 0, AutoRef
};
ModoOperacao  modoOperacao = AutoRef;	//modo de operação em referência de tensão ou referência automática

enum StatusBuzzer
{
	SemAlarme, Alarme1, Alarme2, Inc, Dec, Enter, Volta
};
StatusBuzzer alarmeAtual = SemAlarme;

enum StatusLeitura
{
	Erro = 0, Ok
};
StatusLeitura  estadoComunicacao = Erro;           //se 1 a comunicacao esta ok, se 0 esta com erro.


//    DEFINIÇÕES DE PARÂMETROS 
#define INICIA_COMUNCACAO          99
#define TENSAO_ID                  49
#define CORRENTE_ID                48
#define PARAM_1_SEGUNDO           0xC2F7      // PARA 1 SEGUNDO = 65536-(16MHz/1024/1Hz) = 49911 = 0xC2F7
#define PARAM_0_1_SEGUNDO         0xF9E6      //0x137F      // PARA 0,1 SEGUNDO = 4991 = 0x137F
#define PARAM_VALOR_MINIMO_TENSAO 0.1           // 0.1 V é o valor mínimo que a fonte tem que informar.
#define PARAM_VALOR_MAXIMO_TENSAO 200         // 200 V é o valor máximo de tensão que a fonte vai informar
#define PARAM_VALOR_SP_MAXIMO_TENSAO  99.9
#define PARAM_VALOR_SP_MINIMO_TENSAO  0.1
#define PARAM_VALOR_HI_MAXIMO_TENSAO  9.9
#define PARAM_VALOR_HI_MINIMO_TENSAO  0.1


//     DECLARÇÃO DAS FUNÇÕES
void leituras(void);
void acionamentos(void);
void ihm(void);
void alarmeBuzzer();
void alertaLed(int statusAlerta);
void shieldLCD(void);
int verificaDadosDaLeitura(float valorLido);


//      OUTROS
//SoftwareSerial Serial2(rxPin, txPin); //configurar comunicação


void setup()
{
	//configurar os pinos
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


	//configurar comunicação
	Serial.begin(57600, SERIAL_8E1);
	Serial2.begin(115200);

	//Configurar LCD
	lcd.begin(16, 2);                    //SETA A QUANTIDADE DE COLUNAS(16) E O NÚMERO DE LINHAS(2) DO DISPLAY. EM SUMA: UMA MATRIZ DE 16 COLUNAS E 2 LINHAS
	lcd.setCursor(0, 0);                 //SETA A POSIÇÃO EM QUE O CURSOR INCIALIZA(LINHA 1) 


   // Configuração do timer1 
	TCCR1A = 0;                         //confira timer para operação normal pinos OC1A e OC1B desconectados
	TCCR1B = 0;                         //limpa registrador
	TCCR1B |= (1 << CS10) | (1 << CS12);    // configura prescaler para 1024: CS12 = 1 e CS10 = 1
	TCNT1 = PARAM_0_1_SEGUNDO;          // incia timer com valor para que estouro ocorra em 0,1 segundo
	TIMSK1 |= (1 << TOIE1);             // habilita a interrupção do TIMER1

}

void loop()
{
	leituras();     //ler
	acionamentos(); //agir
	ihm();          //comunicar
}


/*
 *         LEITURAS
 *
 * Funções responsáveis por ler grandezas físicas
 * devem estar aqui;
 *
 */
int tentativas_sem_sucesso;
void leituras(void) {
	if (lerTensao) {                                                              //ler a cada chamada da interrupção com tempo configurado
		Serial.write(INICIA_COMUNCACAO);                                                  //inicia a comunicação
		Serial.write(TENSAO_ID);                                                          //faz a requisição da tensão da fonte
		lerTensao = 0;                                                                    //aguarda o tempo para a próxima leitura
		byte_da_vez = 0;                                                                  //prepara a variável para a lógica seguinte
		if (tentativas_sem_sucesso++ >= 20) {
			tentativas_sem_sucesso = 20;
			estadoComunicacao = Erro;
			char resetCom[] = { 0xFF,0xFF,0xFF,0xFF };
			Serial.write(resetCom);
			Serial2.println("erro");
		}
	}
	else if (Serial.available() > 0) {                                           //se receber algo na serial
		tentativas_sem_sucesso = 0;                                                       //zera alerta de falta de comunicação
		dados_recebidos_da_fonte[byte_da_vez] = Serial.read();                           //armazena os 4 bytes informados
		Serial2.println(dados_recebidos_da_fonte[byte_da_vez]);
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
		case 2: //se for o 3º byte
			byte_recebido_parte1 = dados_recebidos_da_fonte[byte_da_vez];  //salva o 3º byte
			byte_da_vez++;
			break;
		case 3: //se for o 4º byte
			byte_recebido_parte2 = dados_recebidos_da_fonte[byte_da_vez];                          //salva o 4º byte
			const short temp = byte_recebido_parte1 | byte_recebido_parte2 << 8;
			tensao_da_fonte = static_cast<float>(temp);// ((byte_recebido_parte1)+(byte_recebido_parte2 * 256));                 //junta o 3º e 4º byte, como 100 e 10 = 100,10
			tensao_da_fonte = tensao_da_fonte / 10.0;                                                 //aqui tem o valor recebido de tensão da fonte
			Serial2.println(tensao_da_fonte);                                                   //aqui tem o valor recebido de tensão da fonte
			estadoComunicacao = verificaDadosDaLeitura(tensao_da_fonte) ? Ok : Erro;   //retorna 1(ok), caso a leitura esteja dentro da faixa pré-estabelecida
			byte_da_vez++;
			break;
		default:
			//	Serial.flush();
			//	byte_da_vez = 0;
			break;
		}
	}

}


/*
 *         ACIONAMENTO
 *
 * Funções responsáveis por ler grandezas físicas
 * devem estar aqui;
 *
 */
void acionamentos(void) {
	if (estadoComunicacao == Ok) {                                                        //a compensação é ativa a cada leitura correta da tensão
		if (tensao_da_fonte > (param_Setpoint + param_Histerese)) {                 //então verifica se precisa compensar
			digitalWrite(MENOS_PIN, HIGH);
			digitalWrite(MAIS_PIN, LOW);
		}
		else if (tensao_da_fonte < (param_Setpoint - param_Histerese)) {
			digitalWrite(MENOS_PIN, LOW);
			digitalWrite(MAIS_PIN, HIGH);
		}
		else {
			digitalWrite(MENOS_PIN, LOW);
			digitalWrite(MAIS_PIN, LOW);
		}
	}
	else {                                                                        //Se a compensação não estiver ativa, mantem saídas desligadas
		digitalWrite(MENOS_PIN, LOW);
		digitalWrite(MAIS_PIN, LOW);
	}
}



/*
 *         INTERFACE HUMANO MÁQUINA (IHM)
 *
 * Qualquer função responsável por se comunicar com o
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
	shieldLCD();

}



/*
 *                ALERTA BUZZER
 *
 * Avisa com o sinal do buzzer se a conexão com a serial da fonte
 * foi bem sucedida ou não.
 *
 * Tipos de alertas
 * 1 = ok
 * 2 = faixa_tensao_lida = erro
 * 3 = conexão com a serial com problema
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

void alertaLed(const int statusAlerta) {
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
 * Controla o LCD 16x2 e os botões que estão na mesma placa
 *
 */
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

int checaBotao() // Identifica o botão pressionado
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

void shieldLCD() {
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
		switch (_parametro_da_vez)
		{
		case PARAM_PV:
			alarmeAtual = Inc;
			modoOperacao = modoOperacao ? TensaoRef : AutoRef;
			break;
		case PARAM_SP:
			alarmeAtual = Inc;
			if (modoOperacao == TensaoRef)
			{
				if (param_Setpoint <= PARAM_VALOR_SP_MAXIMO_TENSAO)
					param_Setpoint = param_Setpoint + 0.1;
			}
			else
				if (paramTempoEstab <= PARAM_VALOR_SP_MAXIMO_TENSAO)
					paramTempoEstab = paramTempoEstab + 0.1;

			break;
		case PARAM_HI:
			alarmeAtual = Inc;
			if (param_Histerese <= PARAM_VALOR_HI_MAXIMO_TENSAO)
				param_Histerese = param_Histerese + 0.1;
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
			modoOperacao = modoOperacao ? TensaoRef : AutoRef;
			break;
		case PARAM_SP:
			alarmeAtual = Dec;
			if (modoOperacao == TensaoRef)
			{
				if (param_Setpoint > PARAM_VALOR_SP_MINIMO_TENSAO)
					param_Setpoint = param_Setpoint - 0.1;
			}
			else
				if (paramTempoEstab > PARAM_VALOR_SP_MINIMO_TENSAO)
					paramTempoEstab = paramTempoEstab - 0.1;
			break;
		case PARAM_HI:
			alarmeAtual = Dec;
			if (param_Histerese > PARAM_VALOR_HI_MINIMO_TENSAO)
				param_Histerese = param_Histerese - 0.1;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	const auto apaga = apagaPisca();

	lcd.setCursor(0, 0);

	if (modoOperacao == TensaoRef)
	{
		if (_parametro_da_vez == PARAM_PV && apaga)
			lcd.print("   ");
		else
			lcd.print("(A)");
		lcd.print(" ARCO: ");
		lcd.print(tensao_da_fonte, 1);
		lcd.print(" V");
		lcd.setCursor(0, 1);
		lcd.print("Ref:");
		if (_parametro_da_vez == PARAM_SP && apaga)
			lcd.print("     ");
		else
			lcd.print(param_Setpoint, 1);
		if (param_Setpoint >= 10)
			lcd.setCursor(8, 1);
		else
			lcd.setCursor(7, 1);
		lcd.print("V ");
	}
	else
	{
		if (_parametro_da_vez == PARAM_PV && apaga)
			lcd.print("   ");
		else
			lcd.print("(B)");
		lcd.print(" ARCO: ");
		lcd.print(tensao_da_fonte, 1);
		lcd.print(" V");
		lcd.setCursor(0, 1);
		lcd.print("tEst:");
		if (_parametro_da_vez == PARAM_SP && apaga)
			lcd.print("    ");
		else
			lcd.print(paramTempoEstab, 1);
		if (paramTempoEstab >= 10)
			lcd.setCursor(9, 1);
		else
			lcd.setCursor(8, 1);
		lcd.print("s ");
	}
	lcd.setCursor(10, 1);
	lcd.print("+-");
	if (_parametro_da_vez == PARAM_HI && apaga)
		lcd.print("   ");
	else
		lcd.print(param_Histerese, 1);
	lcd.print("V");
}


/*
 *         VERIFICA DADOS DA LEITURA
 *
 * Verifica se o dados que leu de tensão da fonte estão dentro da
 * faixa préviamente configurada.
 */
int verificaDadosDaLeitura(const float valorLido) {
	if (valorLido >= PARAM_VALOR_MINIMO_TENSAO && valorLido <= PARAM_VALOR_MAXIMO_TENSAO)
		return Ok;
	return Erro;
}


/*
 *         INTERRUPÇÃO
 *
 * Interrupção a cada 0,1 segundo para quaisquer base de tempo que o
 * software precise.
 *
 */

ISR(TIMER1_OVF_vect)
{
	lerTensao = 1;          //Inicia uma nova leitura da tensão da fonte	

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
