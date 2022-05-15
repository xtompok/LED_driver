# Testování a ladění driverů

V rámci testování a případného zjednodušení byly provedeny následující úpravy:
 - varianta 1 - odstraněna vybíjecí dioda na gate
 - varianta 2 - původní
 - varianta 3 - odstraněna vybíjecí dioda, odstraněny budící tranzistory

Měřeno bylo s rezistorovou zátěží 3.9 ohmu, napětí 24V, varianta 2, střída
15/100, 40kHz.

Následné měření na osciloskopu ukázalo, že při vypínání tranzistoru dochází k
mohutnému zvonění na odpojovaném pólu, až 90Vpp/17MHz. Při zapínání tranzistoru ke
zvonění nedocházelo. Připojení kondenzátoru na výstupní svorky snížilo zvonění
cca na polovinu, ale kondenzátor se mohutně zahříval. Napájecí napětí
nevykazovalo tak výrazné zvonění. Připojením schottkyho diody antiparalelně na
výstupní svorky snížilo zvonění také cca na polovinu, ale nedocházelo k
přehřívání diody. Navíc se zvonění přeneslo na napájení. Zapojení 1nF keramiky
na napájení pomohlo trochu, paralelní připojení 100nF keramiky pomohlo významně
a zvonění na 17MHz téměř ustalo a objevilo se menší zvonění na cca 1.25MHz. Po
přidání 1uF paralelně na napájení se zvonění ustálilo na cca 7Vpp na výstupních
svorkách a cca 4Vpp na vstupu. Připojení 47u nebo 2200u na vstupní svorky snížilo
zvonění na 3.2Vpp. 

Utlumením zvonění na výstupních svorkách se objevilo zvonění na gate (cca
4Vpp/50Mhz) o délce několika cyklů. Průzkumem bylo zjištěno, že se toto zvonění
vyskytuje po celé desce včetně napájení, výstup z procesoru je čistý a ke
zvonění dojde až později. Zvonění se vyskytuje na konci Millerova plata, když
klesá napětí na gate. 

Ve variantě 1 je zvonění na gate podstatně menší, ale vypínací doba je podstatně
delší. V rámci testované proudové zátěže 6A bylo u varianty 2 patrné mírně vyšší
zahřívání a napájecí zdroj hlásil vyšší spotřebu (17W místo 14W u varianty 1). 
Zahřívání však bylo dostatečně nízké pro trvalý provoz.

Ve variantě 3 ke zvonění nedocházelo, ale protože se gate nabíjí přes velký
rezistor, tak jsou přechodové děje pomalé a docházelo k přehřívání tranzistoru.
Tuto variantu lze dlouhodobě provozovat při špičkových proudech do 3A. 


