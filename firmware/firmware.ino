/*Firmware para conpensação da posição
 * Autor: Alex Sandro Pereira
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
#include <SoftwareSerial.h>
#include <LiquidCrystal.h> 




//     CONFIGURAÇÃO DE PINOS
#define buzzerPin           3 
#define menosPin            47
#define maisPin             49
#define txPin               16
#define rxPin               17
#define ledvermelhoPin      37
#define ledamareloPin       39
#define ledverdePin         41

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);




//     DECLARAÇÃO DAS VARIÁVEIS GLOBAIS
byte  lerTensao;                    //dispara uma nova leitura da tensão pela interrupção
float tensao_da_fonte;              //valor da tensão com casa decimal *10 (Ex: 10,1 V = 101 V)
byte   byte_recebido_parte1;         //os valores recebidos são divididos em dois
byte   byte_recebido_parte2;         //a segunda parte são os decimais
int   dados_recebidos_da_fonte[3];  //recebe os 4 bytes da fonte
byte  compensacao;                  //informa se a compensação esta ativa ou não
byte  faixa_tensao_lida;            //alerta que a tensão da fonte esta fora da faixa informada como min e max
byte  estado_comunicacao;           //se 1 a comunicacao esta ok, se 0 esta com erro.
int   base_tempo1_buzzer;           //base de tempo para o buzzer
int   base_tempo2_buzzer;           //2ª base de tempo para o buzzer
int   base_tempo1_led;               //base de tempo para o led
int   base_tempo2_led;               //2ª base de tempo para o led
int   byte_da_vez;                   //para a lógica de recebimento dos bytes da fonte na função de leitura da tensão
float param_Histerese = 5;           // Valor da histerese = 50 = 5,0 V
float param_Setpoint = 12;          // 60,0 V




//     DEFINIÇÕES DO SISTEMA
#define ok                  1
#define erro                0
#define on                  1
#define off                 0
#define sem_alerta          1
#define alerta1             2 
#define alerta2             3


//    DEFINIÇÕES DE PARÂMETROS 
#define iniciaComuncacao          99
#define tensaoID                  49
#define correnteID                48
#define param_1_segundo           0xC2F7      // PARA 1 SEGUNDO = 65536-(16MHz/1024/1Hz) = 49911 = 0xC2F7
#define param_0_1_segundo         0xF9E6      //0x137F      // PARA 0,1 SEGUNDO = 4991 = 0x137F
#define param_valor_minimo_tensao 1           // 1,0 V é o valor mínimo que a fonte tem que informar.
#define param_valor_maximo_tensao 200         // 200 V é o valor máximo de tensão que a fonte vai informar
#define param_valor_sp_maximo_tensao  1000
#define param_valor_sp_minimo_tensao  0.01
#define param_valor_hi_maximo_tensao  100
#define param_valor_hi_minimo_tensao  0.01




//     DECLARÇÃO DAS FUNÇÕES
void leituras (void);
void acionamentos (void);
void ihm (void);
void alertaBuzzer (int qual_alerta);
void alertaLed (int qual_alerta);
void shieldLCD (void);
byte verifica_dados_da_leitura(int _valor_lido);



//      OUTROS
//SoftwareSerial Serial3(rxPin, txPin); //configurar comunicação


void setup() 
{
    //configurar os pinos
    pinMode(buzzerPin,OUTPUT);   
    pinMode(menosPin,OUTPUT);   
    pinMode(maisPin,OUTPUT);  
    pinMode(ledvermelhoPin,OUTPUT);  
    pinMode(ledamareloPin,OUTPUT);  
    pinMode(ledverdePin,OUTPUT);  
    digitalWrite(buzzerPin, LOW);
    digitalWrite(menosPin, LOW);
    digitalWrite(maisPin, LOW);
    digitalWrite(ledvermelhoPin, LOW);
    digitalWrite(ledamareloPin, LOW);
    digitalWrite(ledverdePin, LOW);
     

    //configurar comunicação
    Serial2.begin(57600, SERIAL_8E1);
    Serial.begin(115200);

    //Configurar LCD
    lcd.begin(16,2);                    //SETA A QUANTIDADE DE COLUNAS(16) E O NÚMERO DE LINHAS(2) DO DISPLAY. EM SUMA: UMA MATRIZ DE 16 COLUNAS E 2 LINHAS
    lcd.setCursor(0,0);                 //SETA A POSIÇÃO EM QUE O CURSOR INCIALIZA(LINHA 1) 
    

   // Configuração do timer1 
    TCCR1A = 0;                         //confira timer para operação normal pinos OC1A e OC1B desconectados
    TCCR1B = 0;                         //limpa registrador
    TCCR1B |= (1<<CS10)|(1 << CS12);    // configura prescaler para 1024: CS12 = 1 e CS10 = 1
    TCNT1 = param_0_1_segundo;          // incia timer com valor para que estouro ocorra em 0,1 segundo
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
void leituras (void){
  if(lerTensao){                                                              //ler a cada chamada da interrupção com tempo configurado
    Serial2.write(iniciaComuncacao);                                                  //inicia a comunicação
    Serial2.write(tensaoID);                                                          //faz a requisição da tensão da fonte
    lerTensao = 0;                                                                    //aguarda o tempo para a próxima leitura
    byte_da_vez = 0;                                                                  //prepara a variável para a lógica seguinte
    if(tentativas_sem_sucesso++ >= 20){
      tentativas_sem_sucesso = 20;
      estado_comunicacao = erro;
      Serial.println("erro");
    }
  }else if (Serial2.available()>0){                                           //se receber algo na serial
    tentativas_sem_sucesso = 0;                                                       //zera alerta de falta de comunicação
    dados_recebidos_da_fonte[byte_da_vez] = Serial2.read();                           //armazena os 4 bytes informados
    Serial.println(dados_recebidos_da_fonte[byte_da_vez]);

    if(byte_da_vez == 2){                                                             //se for o 3º byte
       byte_recebido_parte1 = dados_recebidos_da_fonte[byte_da_vez];                          //salva o 3º byte
    }else if(byte_da_vez == 3){                                                       //se for o 4º byte
       byte_recebido_parte2 = dados_recebidos_da_fonte[byte_da_vez];                          //salva o 4º byte
       tensao_da_fonte = ((byte_recebido_parte1)+(byte_recebido_parte2*256));                 //junta o 3º e 4º byte, como 100 e 10 = 100,10
       tensao_da_fonte = tensao_da_fonte/10;                                                 //aqui tem o valor recebido de tensão da fonte
       Serial.println(tensao_da_fonte);                                                                                 //verificar se o valor confere:
       estado_comunicacao = verifica_dados_da_leitura(tensao_da_fonte);                          //retorna 1(ok), caso a leitura esteja dentro da faixa pré-estabelecida
    }
    //Serial.println(tensao_da_fonte);
    /*if((byte_da_vez == 0) && (dados_recebidos_da_fonte[byte_da_vez] != iniciaComuncacao)){  //byte 0 tem que ser = iniciaComuncacao (99) para receber os outros bytes.
        byte_da_vez = 0;
      }else{
        byte_da_vez++;                                                                    //1 byte por vez
    }
    */
    byte_da_vez++;  
  } 
  
}


/*
 *         ACIONAMENTO
 *        
 * Funções responsáveis por ler grandezas físicas 
 * devem estar aqui;
 * 
 */
void acionamentos (void){
  if(estado_comunicacao == ok){                                                        //a compensação é ativa a cada leitura correta da tensão
      if(tensao_da_fonte > (param_Setpoint + param_Histerese)){                 //então verifica se precisa compensar
        digitalWrite(menosPin, HIGH);
        digitalWrite(maisPin, LOW);
      }else if(tensao_da_fonte < (param_Setpoint - param_Histerese)){
        digitalWrite(menosPin, LOW);
        digitalWrite(maisPin, HIGH);
      }
      else{
        digitalWrite(menosPin, LOW);
        digitalWrite(maisPin, LOW);
      }
  }else{                                                                        //Se a compensação não estiver ativa, mantem saídas desligadas
        digitalWrite(menosPin, LOW);
        digitalWrite(maisPin, LOW);
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
void ihm (void){
  if(estado_comunicacao == ok){
    alertaBuzzer(ok);
    alertaLed(ok);
    shieldLCD();
  }else{
    alertaBuzzer(alerta2);
    alertaLed(alerta2);
    shieldLCD();
  }
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

void alertaBuzzer (int qual_alerta){
   switch(qual_alerta){
    case sem_alerta:
        noTone(buzzerPin);
        alerta_sem_erro = 1;
        temporaria_1_alertaBuzzer = 0;

        
       /* if(temporaria_2_alertaBuzzer++ == 2){
          tone(buzzerPin, 329);
        }else if(temporaria_2_alertaBuzzer == 1000){
          noTone(buzzerPin);
        }else if(temporaria_2_alertaBuzzer == 1500){
          tone(buzzerPin, 329);
        }else if(temporaria_2_alertaBuzzer == 2000){
          noTone(buzzerPin);
        }else if(temporaria_2_alertaBuzzer >= 4500){
          temporaria_2_alertaBuzzer = 4501;
        }
        */
    break;

    case alerta1:
    
    break;

    case alerta2:
       // temporaria_2_alertaBuzzer = 0;
        if(temporaria_1_alertaBuzzer++ == 1){
          tone(buzzerPin, 30.8);
        }else if(temporaria_1_alertaBuzzer == 9){
          noTone(buzzerPin);
        }else if(temporaria_1_alertaBuzzer == 15){
          tone(buzzerPin, 1131.2);//392
        }else if(temporaria_1_alertaBuzzer == 45){
          noTone(buzzerPin);
        }else if(temporaria_1_alertaBuzzer > 45){
          temporaria_1_alertaBuzzer = 46;
        }
        delay(1);
        break;
   }
}

/*
 *                ALERTA LED
 *   
 * 
 */
#define led_verde_ligado          digitalWrite(ledverdePin,HIGH)
#define led_amarelo_ligado        digitalWrite(ledamareloPin,HIGH)
#define led_vermelho_ligado       digitalWrite(ledvermelhoPin,HIGH)
#define led_verde_desligado       digitalWrite(ledverdePin,LOW)
#define led_amarelo_desligado     digitalWrite(ledamareloPin,LOW)
#define led_vermelho_desligado    digitalWrite(ledvermelhoPin,LOW)

void alertaLed (int qual_alerta){
    switch (qual_alerta){
      case ok:
      led_verde_ligado;
      led_amarelo_desligado;
      led_vermelho_desligado;
      break;

      case alerta1:
      break;

      case alerta2:
          if(base_tempo1_led == 0){
            base_tempo1_led = 10;
          }else if(base_tempo1_led > 5){
            led_verde_desligado;
            led_amarelo_ligado;
            led_vermelho_desligado;
          }else{
            led_verde_desligado;
            led_amarelo_desligado;
            led_vermelho_desligado;
          }
      break;
    }
}
 



/*
 *                shieldLCD
 * Controla o LCD 16x2 e os botões que estão na mesma placa
 * 
 */
 int temporaria_1_shieldLCD;
 int temporaria_2_shieldLCD;
 int temporaria_3_shieldLCD;
 int temporaria_4_shieldLCD;
 int temporaria_5_shieldLCD;
 int temporaria_6_shieldLCD;
 int _parametro_da_vez;
 int _botao_select_antes = 0;
 int _botao_select_agora = 0;
 int tempo_botao_pressionado = 15;
void shieldLCD (void){
    #define botaoDireito  1
    #define botaoCima     2
    #define botaoBaixo    3
    #define botaoEsquerda 4
    #define botaoSelect   5
    #define botaoNenhum   6
    #define solto         1
    #define pressionado   0
    #define _paramPV        1
    #define _paramSP        2
    #define _paramHI        3   
    #define _numerodeparam  3   
  
   int _botao_pressionado = 0;



   
   if ((analogRead(0)) < 80) { //SE A LEITURA DO PINO FOR MENOR QUE 80, FAZ 
        if(temporaria_1_shieldLCD++ >= tempo_botao_pressionado){
          temporaria_1_shieldLCD = 0;
          _botao_pressionado = botaoDireito;
          if(tempo_botao_pressionado-- < 1){
            tempo_botao_pressionado = 0;
          }
        }
        temporaria_2_shieldLCD = 0;
        temporaria_3_shieldLCD = 0;
        temporaria_4_shieldLCD = 0;
        temporaria_5_shieldLCD = 0;
   }  
   else if ((analogRead(0)) < 200) { //SE A LEITURA DO PINO FOR MENOR QUE 200, FAZ
        if(temporaria_2_shieldLCD++ >= tempo_botao_pressionado){
            temporaria_2_shieldLCD = 0;
            _botao_pressionado = botaoCima;
            
          if(tempo_botao_pressionado-- < 1){
            tempo_botao_pressionado = 0;
          }
        }
        temporaria_1_shieldLCD = 0;
        temporaria_3_shieldLCD = 0;
        temporaria_4_shieldLCD = 0;
        temporaria_5_shieldLCD = 0;
   }  
   else if ((analogRead(0)) < 400){ //SE A LEITURA DO PINO FOR MENOR QUE 400, FAZ  
        if(temporaria_3_shieldLCD++ >= tempo_botao_pressionado){
            temporaria_3_shieldLCD = 0;
            _botao_pressionado = botaoBaixo;
            
          if(tempo_botao_pressionado-- < 1){
            tempo_botao_pressionado = 0;
          }
        }
        temporaria_1_shieldLCD = 0;
        temporaria_2_shieldLCD = 0;
        temporaria_4_shieldLCD = 0;
        temporaria_5_shieldLCD = 0;
   }  
   else if ((analogRead(0)) < 600){ //SE A LEITURA DO PINO FOR MENOR QUE 600, FAZ  
        if(temporaria_4_shieldLCD++ >= tempo_botao_pressionado){
            temporaria_4_shieldLCD = 0;
            _botao_pressionado = botaoEsquerda;
            
          if(tempo_botao_pressionado-- < 1){
            tempo_botao_pressionado = 0;
          }
        }
        temporaria_1_shieldLCD = 0;
        temporaria_2_shieldLCD = 0;
        temporaria_3_shieldLCD = 0;
        temporaria_5_shieldLCD = 0;
   }  
   else if ((analogRead(0)) < 800){ //SE A LEITURA DO PINO FOR MENOR QUE 800, FAZ 
        if(temporaria_5_shieldLCD++ >= tempo_botao_pressionado){
            temporaria_5_shieldLCD = 0;
            _botao_pressionado = botaoSelect;
        }
        temporaria_1_shieldLCD = 0;
        temporaria_2_shieldLCD = 0;
        temporaria_3_shieldLCD = 0;
        temporaria_4_shieldLCD = 0;
   }
   else{
      _botao_pressionado = botaoNenhum;
      _botao_select_antes = solto;
      tempo_botao_pressionado = 15;
   }



   

   switch (_botao_pressionado){
    case botaoSelect:
        if(_botao_select_antes == solto){
            if(_parametro_da_vez++ > (_numerodeparam-1)){
              _parametro_da_vez = 0;
            }
            _botao_select_antes = pressionado;
        }
    break;
    case botaoDireito:

    break;
    case botaoEsquerda:

    break;
    case botaoCima:
        if(_parametro_da_vez == _paramPV){
          
        }else if(_parametro_da_vez == _paramSP){
          if(param_Setpoint <= param_valor_sp_maximo_tensao){
            param_Setpoint = param_Setpoint + 0.01;
          }
        }else if(_parametro_da_vez == _paramHI){
          if(param_Histerese <= param_valor_hi_maximo_tensao){
              param_Histerese = param_Histerese + 0.01;
          }
        }
    break;
    case botaoBaixo:
        if(_parametro_da_vez == _paramPV){
          
        }else if(_parametro_da_vez == _paramSP){
          if(param_Setpoint > param_valor_sp_minimo_tensao){
              param_Setpoint = param_Setpoint - 0.01;
          }
        }else if(_parametro_da_vez == _paramHI){
          if(param_Histerese > param_valor_hi_minimo_tensao){
            param_Histerese = param_Histerese - 0.1;
          }
        }
    break;
   }

   
   
    lcd.setCursor(0,0);
    if(_parametro_da_vez == _paramPV){
      if(temporaria_6_shieldLCD++ < 40){
        lcd.print ("TENSAO: "); 
      }
      else if(temporaria_6_shieldLCD++ < 80){
        lcd.print ("        ");
      }
      else if(temporaria_6_shieldLCD++ > 81){
        temporaria_6_shieldLCD = 0;
      }
    }else{
      lcd.print ("TENSAO: ");
    }
    lcd.print (tensao_da_fonte);
    lcd.print (" V "); 
    lcd.setCursor(0,1);
    
    if(_parametro_da_vez == _paramSP){
      if(temporaria_6_shieldLCD++ < 40){
        lcd.print ("SP "); 
      }
      else if(temporaria_6_shieldLCD++ < 80){
        lcd.print ("   ");
      }
      else if(temporaria_6_shieldLCD++ > 81){
        temporaria_6_shieldLCD = 0;
      }
    }else{
      lcd.print ("SP "); 
    }
    lcd.print (param_Setpoint);
    lcd.setCursor(9,1);
    
    if(_parametro_da_vez == _paramHI){
      if(temporaria_6_shieldLCD++ < 40){
        lcd.print (" Hi"); 
      }
      else if(temporaria_6_shieldLCD++ < 80){
        lcd.print ("   ");
      }
      else if(temporaria_6_shieldLCD++ > 81){
        temporaria_6_shieldLCD = 0;
      }
    }else{
      lcd.print (" Hi");
    }
    lcd.print (param_Histerese);
}


/*
 *         VERIFICA DADOS DA LEITURA
 *        
 * Verifica se o dados que leu de tensão da fonte estão dentro da
 * faixa préviamente configurada.
 */
byte verifica_dados_da_leitura(float _valor_lido){
  if((_valor_lido >= param_valor_minimo_tensao) && (_valor_lido <= param_valor_maximo_tensao)){
    return ok;
  }else{
    return erro;
  }
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
  if(!lerTensao){
    lerTensao = 1;          //Inicia uma nova leitura da tensão da fonte
  }

  if(base_tempo1_buzzer){
    base_tempo1_buzzer--;
  }
  if(base_tempo2_buzzer){
    base_tempo2_buzzer--;
  }

  if(base_tempo1_led){
    base_tempo1_led--;
  }
  if(base_tempo2_led){
    base_tempo2_led--;
  }
  
  TCNT1 = param_0_1_segundo;   // Renicia TIMER
}
