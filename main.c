/*
met dank aan Xavier van Rijnsoever voor de code voor pwm
 */
#include "h_bridge.h"
#include "servo.h"

#include <avr/io.h>
#include <util/delay.h>

/*
motorDirectie:
0: motor uit
1: motor aan
2: motor aan reverse
*/

int brugStatus = 0; //0: dicht, 1:open
int brugWilStatus = 0;//0: niks, 1: brug wil openen, 2:brug wil dicht

int motorDirectie = 0;
int slagboomDirectie = 0;
int slagboomStatus = 0; //0 is laag, 1 is hoog, 2 is in beweging
int slagboomPositie = 0;
int lichtsignaalStatus = 0;
int vaarverkeerStatus = 0;
int vaarverkeerWachtStatus = 0;
int geluidsignaalStatus = 0;
int noodstopStatus = 0;
int vaarCounter = 0;
int noordBootCounter = 0;
int zuidBootCounter = 0;


unsigned long long int ticks = 0; //1 tick is ongeveer 1ms
unsigned long long int ticksSindsLichtsignaalwissel = 0;
unsigned long long int ticksVoorNoodknop = 0;
unsigned long long int ticksVoorSlagboom = 0;
unsigned long long int ticksMotorBeweging = 0;
unsigned long long int ticksSindsKnop = 0;
unsigned long long int ticksSindsWil = 0;

int motor()
{
    if(motorDirectie == 0){
        //motor uit
        h_bridge_set_percentage(0);
        ticksMotorBeweging = ticks;
    }
    else if(motorDirectie == 1){
        //motor aan
        h_bridge_set_percentage(25);
    }
    else if(motorDirectie == 2){
        //motor aan reverse
        h_bridge_set_percentage(-25);
    }
    if((ticks-ticksMotorBeweging)>10000){
        noodstopStatus = 1;
    }
    return(0);
}

void slagboom()
{
    if((slagboomDirectie == 0) && (slagboomStatus != 0))
        {
        //slagboom omplaag
        servo1_set_percentage(slagboomPositie);
        servo2_set_percentage(slagboomPositie);
        if ((ticks-ticksVoorSlagboom)>=100){
            slagboomPositie += 10;
            ticksVoorSlagboom = ticks;
        }
        slagboomStatus = 2;
        if (slagboomPositie>=100){
            slagboomStatus = slagboomDirectie;
            slagboomPositie = 100;
        }
    }
    else if((slagboomDirectie == 1) && (slagboomStatus != 1))
        {//open
        servo1_set_percentage(slagboomPositie);
        servo2_set_percentage(slagboomPositie);
        if ((ticks-ticksVoorSlagboom)>=100){
            slagboomPositie += -10;
            ticksVoorSlagboom = ticks;
        }
        slagboomStatus = 2;
        if (slagboomPositie<=-100){
            slagboomStatus = slagboomDirectie;
            slagboomPositie = -100;
        }
    }

    if(slagboomStatus == 0){
        PORTL |= 0b00000001;
    }
    else{
        PORTL &= ~0b00000001;
    }
}

/*
    PC7 = weg links
    PC6 = weg rechts
*/

int lichtsignaalWegverkeer()
{
    if((lichtsignaalStatus == 1)&&(ticks-ticksSindsLichtsignaalwissel>1000)) {
        // lichtsignaal tijdelijk uit (knipper uit)
        PORTC &= ~0b11000000;
        ticksSindsLichtsignaalwissel = ticks;
        lichtsignaalStatus = 2;
    }
    else if((lichtsignaalStatus == 2)&&(ticks-ticksSindsLichtsignaalwissel>1000)) {
        // lichtsignaal aan (knipper aan)
        PORTC |= 0b11000000;
        ticksSindsLichtsignaalwissel = ticks;
        lichtsignaalStatus = 1;
    }
    else if(lichtsignaalStatus==0){
        //lichtsignaal uit
        PORTC &= ~0b11000000;
    }
    return(0);
}

/*
Vaarverkeerstatus
	0: geen vaarverkeer van beide zeiden
	1: wel vaarverkeer van beide kanten
	2: wel vaarverkeer van Noordkant
	3: wel vaarverkeer van Zuidkant
	4: aanstonds Noord
	5: aanstonds Zuid
*/

/*
PA0: groen zuid
PA1: groen noord
PA2: geel noord
PA3: geel zuid
PA4: rood zuid
PA5: rood noord
PA6: sensor 1
PA7: sensor 2
*/

int lichtsignaalVaarverkeer()
{
    //Waarde    BootNoord   BootZuid    Oranje
    if(vaarverkeerStatus == 0){
        //0     Rood        Rood        Uit
        PORTA |= 0b00110000;
        PORTA &= ~0b00001111;
        PORTB |= 0b00110000;
        PORTB &= ~0b00001111;
    }
    else if(vaarverkeerStatus == 1){
        //1     Rood        Rood        Aan
        PORTA |= 0b00111100;
        PORTA &= ~0b00000011;
        PORTB |= 0b00110000;
        PORTB &= ~0b00000011;
    }
    else if(vaarverkeerStatus == 2){
        //2     Groen       Rood        Uit
        PORTA |= 0b00010010;
        PORTA &= ~0b00101100;
        PORTB |= 0b00010010;
        PORTB &= ~0b00101100;
     }
    else if(vaarverkeerStatus == 3){
        //3     Rood        Groen       Uit
        PORTA |= 0b00100001;
        PORTA &= ~0b00011110;
        PORTB |= 0b00100001;
        PORTB &= ~0b00011110;
    }
    else if(vaarverkeerStatus == 4){
        //4     Groen&Rood  Rood        Uit
        PORTA |= 0b00110010;
        PORTA &= ~0b00011101;
        PORTB |= 0b00100001;
        PORTB &= ~0b00011110;
    }
    else if(vaarverkeerStatus == 5){
        //5     Rood        Groen&Rood  Uit
        PORTA |= 0b00110001;
        PORTA &= ~0b00001110;
        PORTB |= 0b00100001;
        PORTB &= ~0b00011110;
    }
    return(0);
}

int geluidsignaalWegverkeer()
{
    if (geluidsignaalStatus == 0){
        PORTC &= ~0b00000100;
        PORTK &= ~0b10000000;//PK7 uit
    }
    if (geluidsignaalStatus == 1){
        if((ticks%2)==0){
            PORTC |= 0b00000100;
        }
        else{
            PORTC &= ~0b00000100;
        }
        //Aan
        PORTK |= 0b10000000;//PK7 aan
    }

    return(0);
}

int noodknopUitlezen(){
    return(PINF&0b10000000);
}
int brugOpenenUitlezen(){
    return(PINF&0b01000000);
}
int brugSluitenUitlezen(){
    return(PINF&0b00100000);
}
int brugStopBovenUitlezen(){
    return(!(PINK&0b00000010));
}
int brugStopBenedenUitlezen(){
    return(!(PINK&0b00000001));
}
int bootNoordUitlezen(){
    return(PINF&0b00010000);
}
int bootZuidUitlezen(){
    return(PINF&0b00001000);
}
int bootResetUitlezen(){
    return(PINF&0b00000100);
}
int brugBuitenWerkinguitlezen(){
    return(PINF&0b00000010);
}
int oranjeSwitchUitlezen(){
    return(PINF&0b00000001);
}

int checkNoodknopAan(){
    if(noodknopUitlezen()!=0){
        noodstopStatus = 1;
    }
}
 //moet vervangen worden door brugwil maar is makkelijk om de brug motor te testen
int brugOpenen(){
    if(brugOpenenUitlezen()!=0){//slagboom dicht, geluid aan, lampen knipperen
        //motor aan
        brugWilStatus = 1; //checken
    }
}

int brugSluiten(){
    if(brugSluitenUitlezen()!=0){//voegdelaytoe
        //motor aan reverse
        brugWilStatus = 2;//checken
    }
}

int brugBewegen(){
    brugOpenen();
    brugSluiten();
}

int checkBrugStop(){
    if((motorDirectie == 1)&&(brugStopBovenUitlezen()!=0)){
        motorDirectie = 0;
        brugStatus = 1;
    }
    else if((motorDirectie == 2)&&(brugStopBenedenUitlezen()!=0)){
        motorDirectie = 0;
        brugStatus = 0;
    }
}

int checkBootKantKnoppen(){
    if((bootNoordUitlezen())&&((ticks-ticksSindsKnop)>300)){
        ticksSindsKnop = ticks;
        noordBootCounter++;
    }
    if((bootZuidUitlezen())&&((ticks-ticksSindsKnop)>300)){
        ticksSindsKnop = ticks;
        zuidBootCounter++;
    }
    if(bootResetUitlezen()){
       noordBootCounter = 0;
       zuidBootCounter = 0;
       }
}
int uptdateBootStatusLampen(){
    if(noordBootCounter!=0){
        PORTL |= 0b00010000;//PL4 aan
    }
    else{
        PORTL &= ~0b00010000;//PL4 uit
    }
    if(zuidBootCounter!=0){
        PORTL |= 0b00001000;//PL3 aan
    }
    else{
        PORTL &= ~0b00001000;//PL3 uit
    }

}

int brugOpenSluitSequence(){
    if(brugWilStatus == 0){
        ticksSindsWil = ticks;
    }
    if(brugWilStatus == 1){
        if((ticks-ticksSindsWil)<100){
        lichtsignaalStatus = 2; //wegverkeer
        }
        if((ticks-ticksSindsWil)>3000){
            geluidsignaalStatus = 1;
        }
        if((ticks-ticksSindsWil)>5000){
            slagboomDirectie = 0;
        }
        if((slagboomStatus == 0)&&((ticks-ticksSindsWil)>10000)){
            motorDirectie = 1;
        }
        if(brugStatus == 1){
            geluidsignaalStatus = 0;
            brugWilStatus = 0;
        }
         //geluid

        //servo
        //motor
        //geluid uit
    }
    if(brugWilStatus == 2){
        //lichten bootverkeer uit
        geluidsignaalStatus = 1; //geluid aan
        if(((ticks-ticksSindsWil)>5000)&&(brugStatus != 0)){
            motorDirectie = 2;
        }
        if(brugStatus == 0){
            slagboomDirectie = 1;
        }
        if(slagboomStatus == 1){
            geluidsignaalStatus = 0;
            lichtsignaalStatus = 0;
            brugWilStatus = 0;
        }
        //slagboom omhoog
        //geluid uit
        //licht uit
    }
}
int checkVoorang(){
    if(noordBootCounter>zuidBootCounter){

    }
    if(zuidBootCounter>noordBootCounter){

    }
}

int updateBrugStatus(){
    if(brugStopBenedenUitlezen()){
        PORTL |= 0b00100000;
    }
    else{
        PORTL &= ~0b00100000;
    }
    if(brugStopBovenUitlezen()){
        PORTL |= 0b01000000;
    }
    else{
        PORTL &= ~0b01000000;
    }
}

int updatevaarstatus(){
    if((brugStopBovenUitlezen())&&(noodstopStatus==0)){

        if(noordBootCounter>0){
            vaarverkeerStatus = 2;
        }
        else if(zuidBootCounter>0){
            vaarverkeerStatus = 3;
        }
        else vaarverkeerStatus = 1;
    }
    else if(noodstopStatus==0){
        vaarverkeerStatus = 1;
    }
    else{
        vaarverkeerStatus = 0;
    }



}

int irUitlezen(){
    //sensor 1 aan
    //sensor 1 uit (maakt niet uit)
    //sensor 2 aan
    //sensor 2 uit (brug is vrij) (falling edge detectie) counter -1
    //sensor 1 en 2 omdraaien voor tegengestelde richting


}

int windsensorUitlezen()
{
    return(0);
}

int noodstop()
{
    if (noodstopStatus == 0){
        checkNoodknopAan();
        PORTL &= ~0b10000000;
    }
    if (noodstopStatus == 1){
        brugWilStatus = 0;
        motorDirectie = 0;          //motoruit
        if (slagboomDirectie !=2){
            slagboomDirectie = 0;   //slagbomen omlaag
        }
        //vaarverkeerStatus = 0; //lampen boten rood
        if (lichtsignaalStatus == 0){
            lichtsignaalStatus = 2; //lampen voetgangers aan
        }
        geluidsignaalStatus = 1; //geluid aan
        PORTL |= 0b10000000;//noodknoplampje aan

        if(brugOpenenUitlezen()){
            noodstopStatus = 0;
            brugWilStatus = 1;
        }
        else if(brugSluitenUitlezen()){
            noodstopStatus = 0;
            brugWilStatus = 2;
        }
    }
}




int main(void)
{
    init_h_bridge();
    init_servo();
    DDRC |= 0b11000000;   //wegverkeer lampjes output
    DDRB |= 0b00111111;
    DDRA |= 0b00111111; //lampjes output
    DDRC |= 0b00000100; //lampjes output
    DDRK |= 0b10001100;
    PORTK |= 0b00001111;
    DDRL |= 0b11111111;
    lichtsignaalStatus = 0;
    slagboomDirectie = 1;
    //geluidsignaalStatus = 1;
    while(1){
        //clock
        _delay_ms(1);
        ticks++;
        noodstop();
        motor();
        checkBrugStop();
        lichtsignaalWegverkeer();
        checkBootKantKnoppen();
        geluidsignaalWegverkeer();
        slagboom();
        brugBewegen();
        brugOpenSluitSequence();
        updateBrugStatus();
        updatevaarstatus();
        lichtsignaalVaarverkeer();


        uptdateBootStatusLampen();


        //if(slagboomDirectie)
        /*while(noodstopStatus == 0){
            _delay_ms(1);
            tick++;
            //check noodstop
            //normale brug
            noodstop()

        }
        while(noodstopStatus == 1){
            _delay_ms(1);
             ticks++;

            //check noodstop
            noodstop();
            motorDirectie = 0;
            motor();
            vaarverkeerStatus = 0;//of buiten werking in de toekomst
            lichtsignaalVaarverkeer();

            //noodstop moet aan blijven
        }*/
    }
}
