# Vaja3-ADC-trigger-timer-conversion-STM32F0-Nograsek-Buzina-Kuder


Glede na vašo razvojno ploščico in razširitveno vezje z tipkami ter potenciometri, izberite ustrezni analogni vhod. Kateri pin je to? __PC0__.

Glede na potenciometer na vaši ploščici izberite-obkljukajte ustrezni kanal/pin. Na zaslonu se vam mora usterzno pobarvati izbrani pin v zeleno barvo. Kaj se izpiše poleg pina? __GPIO_Analog ADC1_IN10.

Aktiviramo samo zeleno LED diodo na ustreznem izhodu ___PD12__.

V Clock Configuration spremenimo APB1 Timer clock (MHz) na 16 MHz (pritisnemo ENTER). Kaj opazite? __Vsi ostali razen APB1 peripheral clocks se spremenijo na 32 MHz. APB1 peripheral clocks se spremeni na 8MHz__.

V razdelku TIM1, pod Counter Settings, bi radi časovniku spremenili frekvenco na 1 kHz, zato moramo frekvenco ABP1 Timer Clock preskalirati v polju Prescaler (PSC – 16 bit value). Koliko znaša ta vrednost? ______16__MHz________. 

