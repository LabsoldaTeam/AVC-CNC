#pragma once
#include <EEPROM.h>

enum ModoOperacao
{
	TensaoRef = 0, AutoRef
};
enum EstadoTensao
{
	Off, Estabilizacao, Soldagem
};
enum StatusBuzzer
{
	SemAlarme, Alarme1, Alarme2, Inc, Dec, Enter, Volta
};
enum StatusLeitura
{
	Erro = 0, Ok
};

#define PARAM_VALOR_MINIMO_TENSAO 0.1           // 0.1 V � o valor m�nimo que a fonte tem que informar.
#define PARAM_VALOR_MAXIMO_TENSAO 200         // 200 V � o valor m�ximo de tens�o que a fonte vai informar
#define PARAM_VALOR_SP_MAXIMO_TENSAO  99.9
#define PARAM_VALOR_SP_MINIMO_TENSAO  9.0
#define PARAM_VALOR_ESTAB_MAXIMO  99.9
#define PARAM_VALOR_ESTAB_MINIMO  0
#define PARAM_VALOR_HI_MAXIMO_TENSAO  9.9
#define PARAM_VALOR_HI_MINIMO_TENSAO  0.1

#define PARAM_TENSAO_MINIMA_OFF 15.0 // que aparece quando o arco está apagado (varia de 2 a 40)
#define PARAM_TEMPO_OFF 1.0 //tempo que o sistema espera, após uma tensão de OFF


struct ParamsAvc
{
	float histerese,        // Valor da histerese em V
		setpoint,			// Ref em V
		tempoEstab;			// Tempo de estabiliza��o em s, para atualiza��o autom�tica da refer�ncia
	ModoOperacao modoOperacao;
	void checaErros()
	{
		if (histerese < PARAM_VALOR_HI_MINIMO_TENSAO || histerese > PARAM_VALOR_HI_MAXIMO_TENSAO || isnan(histerese))
			histerese = 0.5;
		if (setpoint < PARAM_VALOR_SP_MINIMO_TENSAO || setpoint > PARAM_VALOR_SP_MAXIMO_TENSAO || isnan(setpoint))
			setpoint = 22.5;
		if (tempoEstab < PARAM_VALOR_ESTAB_MINIMO || tempoEstab > PARAM_VALOR_ESTAB_MAXIMO || isnan(tempoEstab))
			tempoEstab = 3.5;
		if (modoOperacao < 0 || modoOperacao > 1 || isnan(modoOperacao))
			modoOperacao = TensaoRef;
	}
};

ParamsAvc paramsAvc{ 0.5, 12,3.5,AutoRef };
ParamsAvc paramsAvcAnt{ 0.5, 12,3.5,AutoRef };
long tSaveUpdate = 0;					// Momento em que foi checada altera��o nos par�metros

inline void loadParametros()
{
	EEPROM.get(0, paramsAvc);
#ifdef DEBUG_SERIAL
	Serial.print("paramsAvc.modoOperacao:");
	Serial.println(paramsAvc.modoOperacao);
	Serial.print("paramsAvc.histerese:");
	Serial.println(paramsAvc.histerese);
	Serial.print("paramsAvc.setpoint:");
	Serial.println(paramsAvc.setpoint);
	Serial.print("paramsAvc.tempoEstab:");
	Serial.println(paramsAvc.tempoEstab);
#endif
	paramsAvc.checaErros();
}

inline void saveParametros()
{
	if (millis() < tSaveUpdate + 3000)
		return;
	if (memcmp(&paramsAvc, &paramsAvcAnt, sizeof(paramsAvc)))
	{
		EEPROM.put(0, paramsAvc);
#ifdef DEBUG_SERIAL
		Serial.println("Parametros salvos!");
#endif
}
	paramsAvcAnt = paramsAvc;
	tSaveUpdate = millis();
}
