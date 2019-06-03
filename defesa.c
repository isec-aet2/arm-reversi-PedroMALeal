/*função que imprime que está na frente após cada jogada
ScorePlayer1 e ScorePlayer2 são variaveis globais que teem 
guardados os Scores dos jogadores após cada jogada*/

void instanteWinner(){

	char string[50];

	if(ScorePlayer1>ScorePlayer2){
		sprintf(string, "Player in front: Player1");
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_DisplayStringAt(20,300 , (uint8_t *)string, RIGHT_MODE);
	}
	else if(ScorePlayer1<ScorePlayer2){
		sprintf(string, "Player in front: Player2");
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_DisplayStringAt(20,300 , (uint8_t *)string, RIGHT_MODE);
	}
	else if(ScorePlayer1=ScorePlayer2){
		sprintf(string, "Player in front: Both");
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_DisplayStringAt(20,300 , (uint8_t *)string, RIGHT_MODE);
	}
}

/*função que reseta o tempo se o mesmo for pressionado através do Touch Screen
o tempo está a ser "imprimido" da seguinte forma em que countTotal é uma variavel global
	int h=countTotal/3600, m=(countTotal-3600*h)/60, s=(countTotal-3600*h-m*60);
	sprintf(string, "Time Total: %2d:%2d:%2d", h,m,s);
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(20, BSP_LCD_GetYSize()/2 + 120, (uint8_t *)string, RIGHT_MODE);*/

void resetTime(){
	if(flagTS){
		if(TS_State.touchX[0]>=560 && TS_State.touchX[0]<=780 && TS_State.touchY[0]>=350 && TS_State.touchY[0]<=370)
			countTotal=0;
	}
}