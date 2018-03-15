
/**********************************************************************/
/****utiliza los ficheros de texto de los comandos del gnuplot: ***/
/*****  ConfiguraDibujar2.gp y ConfiguraMultiDibujar.gp*/
/**********************************************************************/

#include "plotter.h"


static char *command_c= "/usr/bin/gnuplot> dump1";
#define command_d command_c

static FILE *plot1;

//**************************
//kbhit2():Ãºtil en LINUX; no creo que funcione en WINDOWS. En este caso sustituir kbhit2() por kbhit() en todas partes. 
//ckeyb() ni idea, pero en LINUX funciona

int kbhit2(void){
	struct timeval tv;
		fd_set read_fd;
		tv.tv_sec=0;
		tv.tv_usec=0;
		FD_ZERO(&read_fd);
		FD_SET(0,&read_fd);
		if(select(1, &read_fd, NULL, NULL, &tv) == -1)
			return 0;
				if(FD_ISSET(0,&read_fd))
					return 1;
						return 0;
} 
///////////////

void itoa2(int n, char *CharN) //pasa int a string
{
	char *Aux=CharN,temp;
	short signo=1;

	if(n<0) //si el numero es negativo
	{ --signo; n*=-1; }

	while(n)//escribe el numero al reves
	{
		(*CharN++)=(n%10)+'0'; //lo cambia a caracter
		n/=10;
	}

	if(!signo)
	{ *CharN='-'; CharN++; }

	*CharN='\0'; //pone el final de la cadena
	CharN--; //se retrocede en 1 por el \0

	for(;Aux<CharN;++Aux,--CharN)
	{
		temp=*Aux;
		*Aux=*CharN;
		*CharN=temp;
	}
}
/////////////////
/*
	 mouse_variables

	 When mousing is active, clicking in the active window will set several user variables
	 that can be accessed from the gnuplot command line. The coordinates of the mouse at
	 the time of the click are stored in MOUSE_X MOUSE_Y MOUSE_X2 and MOUSE_Y2. The mouse
	 button clicked, and any meta-keys active at that time, are stored in
	 MOUSE_BUTTON MOUSE_SHIFT MOUSE_ALT and MOUSE_CTRL. These variables are set to
	 undefined at the start of every plot, and only become defined in the event of a
	 mouse click in the active plot window. To determine from a script if the mouse
	 has been clicked in the active plot window, it is sufficient to test for any one
	 of these variables being defined.

	 plot 'something'
	 pause mouse
	 if (defined(MOUSE_BUTTON)) call 'something_else'; \
	 else print "No mouse click."



*/
void demos(void)
{    char llamada[200] = "call  'c:/mobot/mousevariables.dem'  ";

	fprintf(plot1, "%s\n",llamada);
	fflush(plot1);
}


/*
	 char *my_string;
	 
	 asprintf (&my_string, "Being %d is cool, but being free is best of
	 all.", 4);
	 puts (my_string);


*/

void StartDibujar2(int nplot,char *llamada,char *cplot1,int trazo,char xarango[],char yarango[],
		int xntics,int yntics,float xrmin,float xrmax,float yrmin,
		float yrmax,int xcol[],int ycol[],char *titulo,char *filename,
		char *titulo_x,char *titulo_y, char yetiqueta[][50])  
{
	int num_plots=1;
	int ndig;//cifras en rangos

	char espacio[5]= " ";
	char cierre[5]= "\n";

	int i;
	char ccol1[5]="",ccol01[5]="";
	//char ccol2[5]="",ccol02[5]="";
	char cxrmin[10]="",cxrmax[10]="",cyrmin[10]="",cyrmax[10]="";
	char cxntics[10]="",cyntics[10]="";
	//char cplot13[50]=" points "; //impulses, lines, points, boxes, dots
	//char cplot2[200] = "replot  '";//puede evitarse haciendo plot X,Y; replot da problemas con los terminales
	//char cplot15[100]="' axes x2y2 with lines";
	float xrtics,yrtics;
	ndig=8;

	// configuraciÃ³n de la grÃ¡fica
	i=1;
	while ((xcol[i]!=0) && (ycol[i]!=0))
	{
		i++;
	}
	num_plots=i-1;

	//5. ylabel
	// for (i=1;i<=num_plots;i++)
	//{

	if (num_plots==1)
	{
		strcat(cplot1," set ylabel '");
		strcat(cplot1,titulo_y);
		strcat(cplot1,"' ");
		strcat(cplot1,cierre);
	}
	else
	{
		strcat(cplot1,"set  ylabel '");
		strcat(cplot1,titulo_y);
		//strcat(cplot1," 'titulos' ");
		strcat(cplot1,"' ");
		strcat(cplot1,cierre);
	}
	//}

	strcat(cplot1,"plot  ");
	for (i=1;i<=num_plots;i++)
	{
		//1. fichero
		strcat(cplot1," '");
		strcat(cplot1,filename);
		strcat(cplot1,"' ");

		//2. columnas
		strcat(cplot1," using ");
		itoa2(xcol[i],ccol01);
		strcat(cplot1,ccol01);
		strcat(cplot1,":");
		itoa2(ycol[i],ccol1);
		strcat(cplot1,ccol1);
		//3.titulo en y
		strcat(cplot1," title '");
		strcat(cplot1,yetiqueta[i]);
		strcat(cplot1,"' ");

		//4. trazo
		strcat(cplot1," with ");
		if ((trazo>3)|| (trazo<=1)) strcat(cplot1," lines ");
		if (trazo==2) strcat(cplot1," points ");
		if (trazo==3) strcat(cplot1," impulses ");
		if(i!=num_plots) strcat(cplot1,", ");
	}
	strcat(cplot1,cierre);


	//////
	/*
		 if ((col02!=col01) || (col2!=col1))numplot=2;
		 else numplot=1;

		 if (numplot==2){
	//strcat(cplot1,cplot13a);
	strcat(cplot1,espacio);
	strcat(cplot1,coma);
	strcat(cplot1,comilla);
	strcat(cplot1,filename);
	strcat(cplot1,cplot11a);
	itoa2(col02,ccol02);
	strcat(cplot1,ccol02);
	strcat(cplot1,cplot11b);

	itoa2(col2,ccol2);
	strcat(cplot1,ccol2);
	strcat(cplot1,cplot12);
	strcat(cplot1,comilla);
	strcat(cplot1,titulo_y2);
	strcat(cplot1,comilla);
	strcat(cplot1,cplot13);
	//strcat(cplot1,cplot15);
	strcat(cplot1,cierre2);
	}
	else strcat(cplot1,cierre2);
	*/
	//// fin de plot


	///// call
	//1.titulo x
	strcat(llamada,espacio);
	strcat(llamada,"1"); //$0  nplot=1: plot xy; 2: x2y2
	strcat(llamada,espacio);
	strcat(llamada,"'");
	strcat(llamada,titulo);//$1
	strcat(llamada,"'");
	strcat(llamada,espacio);
	strcat(llamada,titulo_x);//$2
	strcat(llamada,espacio);

	//2.rangos x-y          
	strcat(llamada,espacio);
	if (strcmp(xarango,"*")==0){
		strcat(llamada,espacio);
		strcat(llamada,"'*'"); //$3
		strcat(llamada,espacio);
		strcat(llamada,"'*'"); //$4
		strcat(llamada,espacio);
	}
	else{
		cxrmin[0]='\0';
		strcat(llamada,"'");
		/*        if (xrmin==0) {strcat(cxrmin,"0");}
							else {itoa2(xrmin,cxrmin);}*/
		gcvt(xrmin,ndig,cxrmin);
		//printf("xmin=%i cxmin=%s\n",xrmin,cxrmin);
		strcat(llamada,cxrmin); //$3
		strcat(llamada,"'");
		strcat(llamada,espacio);
		cxrmax[0]='\0';
		/*        if (xrmax==0) {strcat(cxrmax,"0");}
							else {itoa2(xrmax,cxrmax);}*/
		gcvt(xrmax,ndig,cxrmax);
		//printf("xmax=%i cxmax=%s\n",xrmax,cxrmax);
		strcat(llamada,"'");
		strcat(llamada,cxrmax);//$4
		strcat(llamada,"'");
		strcat(llamada,espacio);
	}

	if (strcmp(yarango,"*")==0){
		strcat(llamada,espacio);
		strcat(llamada,"'*'"); //$5
		strcat(llamada,espacio);
		strcat(llamada,"'*'"); //$6
		strcat(llamada,espacio);
	}
	else
	{
		cyrmin[0]='\0';
		/*        if (yrmin==0) {strcat(cyrmin,"0");}
							else {itoa2(yrmin,cyrmin);}*/
		gcvt(yrmin,ndig,cyrmin);
		strcat(llamada,"'");
		strcat(llamada,cyrmin);//$5
		strcat(llamada,"'");
		strcat(llamada,espacio);
		cyrmax[0]='\0';
		/*        if (yrmax==0) {strcat(cyrmax,"0");}
							else {itoa2(yrmax,cyrmax);}*/
		gcvt(yrmax,ndig,cyrmax);
		strcat(llamada,"'");
		strcat(llamada,cyrmax);//$6
		strcat(llamada,"'");
		strcat(llamada,espacio);
	}
	if (strcmp(xarango,"*")==0){
		strcat(llamada,"'");
		strcat(llamada,"1");//$7
		strcat(llamada,"'");
		strcat(llamada,espacio);
	} else {
		strcat(llamada,espacio);
		strcat(llamada,"'");
		if (xntics==0) {strcat(cxntics,"1");}
		else {
			xntics=xntics+1;
			xrtics=(xrmax-xrmin)/xntics;
			gcvt(xrtics,ndig,cxntics);
		}
		strcat(llamada,cxntics);//$7
		strcat(llamada,"'");
		strcat(llamada,espacio);
	}
	if (strcmp(yarango,"*")==0){
		strcat(llamada,"'");
		strcat(llamada,"1");//$8
		strcat(llamada,"'");
		strcat(llamada,espacio);
	} else {
		strcat(llamada,"'");
		if (yntics==0) {strcat(cyntics,"1");}
		else {
			yntics=yntics+1;
			yrtics=(yrmax-yrmin)/yntics;
			gcvt(yrtics,ndig,cyntics);
		}
		strcat(llamada,cyntics);//$8
		strcat(llamada,"'");
		strcat(llamada,espacio);
	}

	if ((strcmp(xarango,"*")==0)&& (strcmp(yarango,"*")==0)){
		strcat(llamada,"'");
		strcat(llamada,"4");//$9
		strcat(llamada,"'");
		strcat(llamada,espacio);
	}
	if  ((strcmp(yarango,"*")==0)&&( strcmp(xarango,"*")!=0)){
		strcat(llamada,"'");
		strcat(llamada,"1");//$9
		strcat(llamada,"'");
		strcat(llamada,espacio);
	}
	if  ((strcmp(yarango,"*")!=0)&&(strcmp(xarango,"*")==0)){
		strcat(llamada,"'");
		strcat(llamada,"3");//$9
		strcat(llamada,"'");
		strcat(llamada,espacio);
	}
	if  ((strcmp(yarango,"*")!=0)&&( strcmp(xarango,"*")!=0)){
		strcat(llamada,"'");
		strcat(llamada,"2");//$9
		strcat(llamada,"'");
		strcat(llamada,espacio);
	}



	strcat(llamada,cierre);

}

void ejecucion_plot(int multip,char *llamada,char *cplot1)
{
	// ejecucion
	fprintf(plot1, "%s",llamada);
	fflush(plot1);
	/*if (multip==2){
		fprintf(plot1,"%s",llamada);
		fflush(plot1);
		}
		*/
	fprintf(plot1, "%s",cplot1);
	//        fprintf(plot1,"save 'conf1.gnu'\n");
	fflush(plot1);
}

void StartMultiDibujar(int num_graf, int marcos,char *llamada)  //multiplot. Se puede hacer con reread probablemente mejor
{
	//char mllamada1[200]="";
	char cngraf[10]="";

	//strcat(llamada,mllamada);
	strcat(llamada," ");
	itoa2(num_graf,cngraf);
	strcat(llamada," ");
	strcat(llamada,cngraf);
	strcat(llamada," ");
	if (marcos==1)
	{
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"1");
		strcat(llamada,"'");
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"0.5");
		strcat(llamada,"'");
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"0");
		strcat(llamada,"'");
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"0.5");
		strcat(llamada,"'");
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"0");
		strcat(llamada,"'");
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"0");
		strcat(llamada,"'");
		strcat(llamada," ");
	}
	if (marcos==2)
	{
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"0.5");
		strcat(llamada,"'");
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"1");
		strcat(llamada,"'");
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"0");
		strcat(llamada,"'");
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"0");
		strcat(llamada,"'");
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"0.5");
		strcat(llamada,"'");
		strcat(llamada," ");
		strcat(llamada,"'");
		strcat(llamada,"0");
		strcat(llamada,"'");
		strcat(llamada," ");
	}


	strcat(llamada,"\n");
}

/////////////////
void config_imp(int opc,char *tipo, char *filename)
{
	char termin_imp[200] = "set terminal ";
	char salida[200] = "set output '";
	char cierre0[5]="\n";
	char cierre1[5]= "' ";
	char cierre2[5]= ".";
	//char ps2[50]="postscript";

	//if (opc==4){ strcat(termin_imp,ps2);}
	//else {strcat(termin_imp,tipo);}
	strcat(termin_imp,tipo);
	//strcat(termin_imp,tipo);
	strcat(termin_imp,cierre0);
	if (filename[0]=='\0'){
		printf("nombre de fichero:");
		scanf("%s",filename);
	}
	strcat(salida,filename);
	strcat(salida,cierre2);
	strcat(salida,tipo);
	strcat(salida,cierre1);
	strcat(salida,cierre0);
	fprintf(plot1, "%s",termin_imp);
	fprintf(plot1, "%s",salida);
}


void terminales(int opc,char filename[50])
{
	char termin1[200] = "set terminal x11\n";
	char termin_W[200] = "set terminal windows 'color'\n";
	char gif1[100]="gif ";
	char png1[10]="png";
	//char ps1[10]="ps";
	//char pdf1[10]="pdf";


	if (fopen(command_d, "r") == '\0')
	{//printf("error d\n");
		plot1 = popen(command_c, "w"); }
			if (fopen(command_c, "r")== '\0')
			{//printf("error c\n");
				plot1 = popen(command_d, "w");}


					switch(opc)
					{
						case 0:fprintf(plot1, "%s",termin_W);
									 break;
						case 1:fprintf(plot1, "%s",termin1);
									 break;
						case 2:config_imp(2,gif1,filename);
									 break;
						case 3:config_imp(3,png1,filename);
									 break;
									 //	case 4:config_imp(4,ps1,filename);
									 //	break;
									 //	case 5:config_imp(5,pdf1,filename);
									 //	break;
					}

}

//////////////



/////////

int dibujar(void)
{
	//gnuplot
	char filename[50]="";
		char conf1[50]="";
		int tipo_plot;
		int xcol[10],ycol[10];
		int i,j,k,sal;
		char xarango[5]="",yarango[5]="",atitulos[5]="";
		float xrmin=0.0,xrmax=0.0,yrmin=0.0,yrmax=0.0;
		int xntics,yntics;
		int opc_terminal=1;
		int numgraf=1;
		char ytitulo[50]="";
		char xtitulo[50]="";
		char yetiqueta[5][50];
		
		char dllamada1[500] = "call  'ConfiguraDibujar1.gp'  ";
		char dmllamada[500] = "call  'ConfiguraMultiDibujar.gp'  ";
	char cplotm[5][500];
	char llamadam[5][500];
	int marcos,tit,trazo;
	char titulo[150]="";
	char cc;
	/////
	// contador();
	//return 15;
	int graf_default=1;


	for (i=0;i<5;i++){
		cplotm[i][0]='\0';
			llamadam[i][0]='\0';
			yetiqueta[i][0]='\0';
	}
	////
	printf("\n\n\n0:salir\n");
		printf("1:plot\n");
		printf("2:multiplot (no admite raton)\n");
		printf("3: presenta fichero configuracion(<FILENAME>.gnu)(¡no funciona en multiplot!)\n");
		
		printf("tipo= ");
		scanf("%i",&tipo_plot );
		if (tipo_plot==0) return 0;
			

				if (tipo_plot==3) {
					printf("NOMBRE DE FICHERO DE DATOS: ");
						scanf("%s", filename);
						strcat(conf1,"load '");
						strcat(conf1,filename);
						strcat(conf1,".gnu' \n");
				}
	if (tipo_plot!=3)
	{
		printf("NOMBRE DE FICHERO DE DATOS: ");
			scanf("%s", filename);
		printf("1.grafico por defecto\n");
			printf("2.grafico con opciones de presentacion\n");
		printf("opc= ");
		scanf("%i",&graf_default);
		if (tipo_plot==1) {printf("MODO SUPERPOSICION con el mismo ejes X\n");numgraf=1;}
		if (tipo_plot==2) {
			printf("MODO MULTIPLOT\n");
				numgraf=2;
				printf("1: arriba/abajo\n");
				printf("2: izquierda/derecha\n");
				scanf("%i",&marcos);
		}
		if (graf_default==1)
		{
			opc_terminal=1;
			tit=1;
			trazo=1;
			atitulos[0]='N';
			xarango[0]='S';
			yarango[0]='S';
		}
		else
		{
			printf("opciones terminal \n");
				printf("1:terminal x11\n");
				printf("2:imagen gif\n");
				printf("3:imagen png y eps\n");
				//		printf("4:terminal impresion postscript\n");
				//		printf("5:terminal impresion pdf (NO FUNCIONA)\n");
				printf("opc= ");
				scanf("%i", &opc_terminal);
			printf("1: TITULO nombre de fichero\n");
				printf("2: TITULO que se quiera\n ");
				printf("opcion= ");
				scanf("%i", &tit);
			

		}
		
			//1.
			titulo[0]='\0';
			if (tit==1) {
				strcat(titulo,filename);
			}else {
				printf(" >> titulo=  ");
					scanf("%s", titulo);
			}  
		
			for (k=1;k<=numgraf;k++)
			{
				if (numgraf>1) printf("\n\n grafica %i",k);
					printf("\n\n");
						if (graf_default==2)
						{                
							printf("\n ESTILO DEL TRAZO \n");
								printf("1. lineas\n");
								printf("2. puntos\n");
								printf("3. impulsos\n");
								printf(" trazo= ");
								scanf("%i",&trazo);
								printf("\n  ¿TITULOS EN LOS EJES X-Y? (S/N) ");
								scanf("%s",atitulos);
						}
				if ((strcmp(atitulos,"S")==0)||(strcmp(atitulos,"s")==0)){
					printf("titulo_x: ");
						scanf("%s", xtitulo);
						printf("titulo_y: ");
						scanf("%s", ytitulo);
				} else {
					atitulos[0]='\0';
						xtitulo[0]='\0';
						ytitulo[0]='\0';
						strcat(atitulos,"*");
						strcat(xtitulo,"*");
						strcat(ytitulo,"*");
				}
				if (graf_default==2)
				{                
					printf("\n   RANGOS DE DIVISION EN LOS EJES X-Y\n");
						printf("\n > ¿AUTOAJUSTE EN X? (S/N) ");
						scanf("%s",xarango);
				}
				if ((strcmp(xarango,"N")==0)||(strcmp(xarango,"n")==0)){
					printf (" >> X min= ");
						scanf("%f",&xrmin);
						printf (" >> X max= ");
						scanf("%f",&xrmax);
						printf(" >> divisiones= ");
						scanf("%i",&xntics);
				}else {xarango[0]='\0';strcat(xarango,"*");}
				if (graf_default==2)
				{                
					printf("\n > ¿AUTOAJUSTE EN Y? (S/N) ");
						scanf("%s",yarango);
				}
				if ((strcmp(yarango,"N")==0)||(strcmp(yarango,"n")==0)){
					printf (" >> Y min= ");
						scanf("%f",&yrmin);
						printf (" >> Y max= ");
						scanf("%f",&yrmax);
						printf(" >> divisiones= ");
						scanf("%i",&yntics);
				}else {yarango[0]='\0';strcat(yarango,"*");}
					
						i=0;
						sal=1;
						xcol[1]=0;
						ycol[1]=0;
						printf("\n   GRAFICAS. x:y:leyenda  (0:SALIR) \n" );
						while (sal!=0){
							i++;
								printf (" >> x%i= ",i);
								scanf("%i",&xcol[i]);
								if (xcol[i]!=0){
									printf (" >> y%i= ",i);
										scanf("%i",&ycol[i]);
										if (ycol[i]!=0){
											printf(" >> leyenda y%i= ",i);
												scanf("%s",yetiqueta[i]);
										}
										else  sal=0;
								} else sal=0;
									
						}
				strcat(llamadam[k],dllamada1);
					StartDibujar2(1,llamadam[k],cplotm[k],trazo,xarango,yarango,xntics,
							yntics,xrmin,xrmax,yrmin,yrmax,xcol,ycol,titulo,filename,xtitulo, ytitulo,yetiqueta);
					if (numgraf>1) {
						strcat(llamadam[k],dmllamada);
							StartMultiDibujar(k,marcos,llamadam[k]);
					}
			}
	}
	
		terminales(opc_terminal,filename);
		
		if (tipo_plot==2) {
			fprintf(plot1,"set multiplot\n");
				fflush(plot1);
		}
	if (tipo_plot!=3)
	{
		for (i=1;i<=numgraf;i++)
		{
			ejecucion_plot(numgraf,llamadam[i],cplotm[i]);
		}
	}
	if (tipo_plot==2) {
		fprintf(plot1,"unset multiplot\n");
			fflush(plot1);
	}
	
		if (tipo_plot==3) 
		{
			fprintf(plot1,conf1);
				fflush(plot1);
				//        if (opc_terminal!=0) pclose(plot1);
		}
		else
		{
			strcat(conf1,"save '");
				strcat(conf1,filename);
				strcat(conf1,".gnu' \n");
				fprintf(plot1,conf1);
				fflush(plot1);
			/*        conf1[0]=NULL;
								strcat(conf1,"call 'ConfiguraRatonTeclas.gp' 'kkf'\n");
								fprintf(plot1,conf1);
								fflush(plot1);
								*/
		}
	
		while(!kbhit2()){}//
	
		//if (tipo_plot!=3)
		pclose(plot1);

	if (opc_terminal==3)
	{
		char filename1[50]="";
		char filename2[50]="";
		char *tira [] = {"sam2p", filename1,"EPS:",filename2,NULL};
		
		strcat(filename1,filename);
		strcat(filename1,".png");
		strcat(filename2,filename);
		strcat(filename2,".eps");

		execv("/usr/bin/sam2p",tira);
	}
	return 0;
}

int main(void)
{
	int err;
		printf("\n\n     j:ejes; g:rejilla\n");
		printf("     RATON: (B3:ZOOM; ESC:salir,n:siguiente,p:previo,u:deshacer zoom)\n");
		printf("            (B1:copiar)(B2:marcar)\n\n");
		err=dibujar();
		if (err!=0) printf("error %i en dibujar",err);
			return 0;
}


