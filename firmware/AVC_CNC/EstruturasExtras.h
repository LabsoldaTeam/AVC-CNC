#pragma once
#include <EEPROM.h>

enum ModoOperacao
{
	TensaoRef = 0, AutoRef
};
enum EstadoAutoRef
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

#define PARAM_VALOR_MINIMO_TENSAO 0.1           // 0.1 V é o valor mínimo que a fonte tem que informar.
#define PARAM_VALOR_MAXIMO_TENSAO 200         // 200 V é o valor máximo de tensão que a fonte vai informar
#define PARAM_VALOR_SP_MAXIMO_TENSAO  99.9
#define PARAM_VALOR_SP_MINIMO_TENSAO  0.1
#define PARAM_VALOR_HI_MAXIMO_TENSAO  9.9
#define PARAM_VALOR_HI_MINIMO_TENSAO  0.1

struct ParamsAvc
{
	float histerese,        // Valor da histerese em V
		setpoint,			// Ref em V
		tempoEstab;			// Tempo de estabilização em s, para atualização automática da referência
	ModoOperacao modoOperacao;
	void checaErros()
	{
		if (histerese < PARAM_VALOR_HI_MINIMO_TENSAO || histerese > PARAM_VALOR_HI_MAXIMO_TENSAO)
			histerese = 0.5;
		if (setpoint < PARAM_VALOR_SP_MINIMO_TENSAO || setpoint > PARAM_VALOR_SP_MAXIMO_TENSAO)
			setpoint = 12.5;
		if (tempoEstab < PARAM_VALOR_SP_MINIMO_TENSAO || tempoEstab > PARAM_VALOR_SP_MAXIMO_TENSAO)
			tempoEstab = 3.5;
	}
};

ParamsAvc paramsAvc{ 0.5, 12,3.5,AutoRef };
ParamsAvc paramsAvcAnt = paramsAvc;
long tSaveUpdate = 0;					// Momento em que foi checada alteração nos parâmetros

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
#ifdef DEBUG_SERIAL
		Serial.println("Parametros salvos!");
#endif
		EEPROM.put(0, paramsAvc);
	}
	paramsAvcAnt = paramsAvc;
	tSaveUpdate = millis();
}