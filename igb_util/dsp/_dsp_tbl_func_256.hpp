#pragma once

constexpr inline float dsp_func_sigmoid_tbl[] = {
  -1.00000000000000000000,
  -0.99883284378421155481,
  -0.99762989155323811286,
  -0.99639009057766914079,
  -0.99511236002779934751,
  -0.99379559040684617433,
  -0.99243864298512773914,
  -0.99104034923625450304,
  -0.98959951027647141686,
  -0.98811489630836835119,
  -0.98658524607027076048,
  -0.98500926629271290391,
  -0.98338563116349486659,
  -0.98171298180292743130,
  -0.97998992575097398916,
  -0.97821503646811380772,
  -0.97638685285186010976,
  -0.97450387877099453604,
  -0.97256458261969713774,
  -0.97056739689388527026,
  -0.96851071779220554436,
  -0.96639290484426054739,
  -0.96421228056879582091,
  -0.96196713016471613322,
  -0.95965570123795029822,
  -0.95727620356733655882,
  -0.95482680891285554026,
  -0.95230565086969809485,
  -0.94971082477181301140,
  -0.94704038764874620693,
  -0.94429235823974255748,
  -0.94146471706924927947,
  -0.93855540658812119936,
  -0.93556233138499522717,
  -0.93248335847245689134,
  -0.92931631765278843726,
  -0.92605900196823487391,
  -0.92270916824087967356,
  -0.91926453770736449300,
  -0.91572279675381929032,
  -0.91208159775650288204,
  -0.90833856003376356458,
  -0.90449127091503578324,
  -0.90053728693267887095,
  -0.89647413514253504463,
  -0.89229931457913813642,
  -0.88801029785154050877,
  -0.88360453288573437280,
  -0.87907944481963062877,
  -0.87443243805651604816,
  -0.86966089848283645303,
  -0.86476219585604763296,
  -0.85973368636813496391,
  -0.85457271539021917306,
  -0.84927662040344831240,
  -0.84384273412110077928,
  -0.83826838780651924399,
  -0.83255091479111997810,
  -0.82668765419631162761,
  -0.82067595486267741478,
  -0.81451317948924339962,
  -0.80819670898505890921,
  -0.80172394703465976740,
  -0.79509232487825598490,
  -0.78829930630669875491,
  -0.78134239287041939992,
  -0.77421912930059988422,
  -0.76692710913984096788,
  -0.75946398057851471908,
  -0.75182745249185334302,
  -0.74401530067161480986,
  -0.73602537424488767570,
  -0.72785560227125700727,
  -0.71950400050815532804,
  -0.71096867833275556592,
  -0.70224784580726273830,
  -0.69333982087289602880,
  -0.68424303665626706739,
  -0.67495604887022997431,
  -0.66547754328963981685,
  -0.65580634328079190354,
  -0.64594141736167065559,
  -0.63588188676848467207,
  -0.62562703300235678050,
  -0.61517630532845557489,
  -0.60452932819834537259,
  -0.59368590856487701846,
  -0.58264604305758793679,
  -0.57140992498532827693,
  -0.55997795113170045767,
  -0.54835072830791820042,
  -0.53652907962686224863,
  -0.52451405046146237776,
  -0.51230691405009332051,
  -0.49990917671142298673,
  -0.48732258263115635355,
  -0.47454911818334488682,
  -0.46159101574943839363,
  -0.44845075699901681965,
  -0.43513107559719310657,
  -0.42163495930501382336,
  -0.40796565144082508336,
  -0.39412665167250027221,
  -0.38012171611265954674,
  -0.36595485669154159680,
  -0.35163033978500624066,
  -0.33715268407824416919,
  -0.32252665764914001389,
  -0.30775727425884513710,
  -0.29284978884097523366,
  -0.27780969218489282024,
  -0.26264270481277951497,
  -0.24735477005458050836,
  -0.23195204632941857081,
  -0.21644089864665294609,
  -0.20082788934439821382,
  -0.18511976808794405880,
  -0.16932346115513152718,
  -0.15344606004025440615,
  -0.13749480941245884047,
  -0.12147709446886023610,
  -0.10540042772662350234,
  -0.08927243530205652977,
  -0.07310084272826244955,
  -0.05689346036609543855,
  -0.04065816846599834555,
  -0.02440290194075334096,
  -0.00813563491122359927,
  0.00813563491122359927,
  0.02440290194075322994,
  0.04065816846599812351,
  0.05689346036609532753,
  0.07310084272826267160,
  0.08927243530205641875,
  0.10540042772662361337,
  0.12147709446886012508,
  0.13749480941245906251,
  0.15344606004025407309,
  0.16932346115513130513,
  0.18511976808794394778,
  0.20082788934439843587,
  0.21644089864665327916,
  0.23195204632941845979,
  0.24735477005458061939,
  0.26264270481277973701,
  0.27780969218489248718,
  0.29284978884097490059,
  0.30775727425884502608,
  0.32252665764914034696,
  0.33715268407824461327,
  0.35163033978500601862,
  0.36595485669154137476,
  0.38012171611265932469,
  0.39412665167250038323,
  0.40796565144082475030,
  0.42163495930501349029,
  0.43513107559719355066,
  0.44845075699901704169,
  0.46159101574943828261,
  0.47454911818334477580,
  0.48732258263115646457,
  0.49990917671142298673,
  0.51230691405009332051,
  0.52451405046146248878,
  0.53652907962686247068,
  0.54835072830791853349,
  0.55997795113170023562,
  0.57140992498532838795,
  0.58264604305758815883,
  0.59368590856487690743,
  0.60452932819834526157,
  0.61517630532845535285,
  0.62562703300235700254,
  0.63588188676848500513,
  0.64594141736167087764,
  0.65580634328079212558,
  0.66547754328964003889,
  0.67495604887022997431,
  0.68424303665626684534,
  0.69333982087289558471,
  0.70224784580726296035,
  0.71096867833275578796,
  0.71950400050815543906,
  0.72785560227125722932,
  0.73602537424488745366,
  0.74401530067161480986,
  0.75182745249185334302,
  0.75946398057851460806,
  0.76692710913984107890,
  0.77421912930059999525,
  0.78134239287041928890,
  0.78829930630669875491,
  0.79509232487825598490,
  0.80172394703465976740,
  0.80819670898505902024,
  0.81451317948924328860,
  0.82067595486267741478,
  0.82668765419631151659,
  0.83255091479112008912,
  0.83826838780651913297,
  0.84384273412110077928,
  0.84927662040344831240,
  0.85457271539021917306,
  0.85973368636813485288,
  0.86476219585604763296,
  0.86966089848283645303,
  0.87443243805651582612,
  0.87907944481963062877,
  0.88360453288573403974,
  0.88801029785154028673,
  0.89229931457913824744,
  0.89647413514253471156,
  0.90053728693267864891,
  0.90449127091503589426,
  0.90833856003376367561,
  0.91208159775650265999,
  0.91572279675381906827,
  0.91926453770736449300,
  0.92270916824087967356,
  0.92605900196823487391,
  0.92931631765278854829,
  0.93248335847245678032,
  0.93556233138499522717,
  0.93855540658812119936,
  0.94146471706924939049,
  0.94429235823974244646,
  0.94704038764874609591,
  0.94971082477181312242,
  0.95230565086969787281,
  0.95482680891285531821,
  0.95727620356733655882,
  0.95965570123795007618,
  0.96196713016471613322,
  0.96421228056879582091,
  0.96639290484426076944,
  0.96851071779220521130,
  0.97056739689388504821,
  0.97256458261969691570,
  0.97450387877099453604,
  0.97638685285185999874,
  0.97821503646811369670,
  0.97998992575097432223,
  0.98171298180292732027,
  0.98338563116349475557,
  0.98500926629271301493,
  0.98658524607027087150,
  0.98811489630836835119,
  0.98959951027647141686,
  0.99104034923625472508,
  0.99243864298512773914,
  0.99379559040684628535,
  0.99511236002779956955,
  0.99639009057766925181,
  0.99762989155323800183,
  0.99883284378421155481,
  1.00000000000000000000,
};
constexpr inline float dsp_func_sinusoid_tbl[] = {
  0.00000000000000000000,
  0.02454122852291228812,
  0.04906767432741801493,
  0.07356456359966742631,
  0.09801714032956060363,
  0.12241067519921619566,
  0.14673047445536174793,
  0.17096188876030121717,
  0.19509032201612824808,
  0.21910124015686979759,
  0.24298017990326387094,
  0.26671275747489836538,
  0.29028467725446233105,
  0.31368174039889151761,
  0.33688985339222005111,
  0.35989503653498811087,
  0.38268343236508978178,
  0.40524131400498986100,
  0.42755509343028208491,
  0.44961132965460653965,
  0.47139673682599764204,
  0.49289819222978403790,
  0.51410274419322166128,
  0.53499761988709715332,
  0.55557023301960217765,
  0.57580819141784533866,
  0.59569930449243335691,
  0.61523159058062681925,
  0.63439328416364548779,
  0.65317284295377675551,
  0.67155895484701833009,
  0.68954054473706682948,
  0.70710678118654746172,
  0.72424708295146689174,
  0.74095112535495899486,
  0.75720884650648445646,
  0.77301045336273688235,
  0.78834642762660622761,
  0.80320753148064483184,
  0.81758481315158371139,
  0.83146961230254512465,
  0.84485356524970700587,
  0.85772861000027211809,
  0.87008699110871134952,
  0.88192126434835493853,
  0.89322430119551532446,
  0.90398929312344333820,
  0.91420975570353069095,
  0.92387953251128673848,
  0.93299279883473884567,
  0.94154406518302080631,
  0.94952818059303667475,
  0.95694033573220893540,
  0.96377606579543984022,
  0.97003125319454397424,
  0.97570213003852857003,
  0.98078528040323043058,
  0.98527764238894122162,
  0.98917650996478101444,
  0.99247953459870996706,
  0.99518472667219681771,
  0.99729045667869020697,
  0.99879545620517240501,
  0.99969881869620424997,
  1.00000000000000000000,
  0.99969881869620424997,
  0.99879545620517240501,
  0.99729045667869020697,
  0.99518472667219692873,
  0.99247953459870996706,
  0.98917650996478101444,
  0.98527764238894122162,
  0.98078528040323043058,
  0.97570213003852857003,
  0.97003125319454397424,
  0.96377606579543984022,
  0.95694033573220893540,
  0.94952818059303667475,
  0.94154406518302080631,
  0.93299279883473884567,
  0.92387953251128673848,
  0.91420975570353069095,
  0.90398929312344344922,
  0.89322430119551521344,
  0.88192126434835504956,
  0.87008699110871146054,
  0.85772861000027211809,
  0.84485356524970722791,
  0.83146961230254545772,
  0.81758481315158371139,
  0.80320753148064494287,
  0.78834642762660633863,
  0.77301045336273710440,
  0.75720884650648478953,
  0.74095112535495899486,
  0.72424708295146700276,
  0.70710678118654757274,
  0.68954054473706705153,
  0.67155895484701855214,
  0.65317284295377664449,
  0.63439328416364548779,
  0.61523159058062693028,
  0.59569930449243346793,
  0.57580819141784544968,
  0.55557023301960217765,
  0.53499761988709715332,
  0.51410274419322177231,
  0.49289819222978414892,
  0.47139673682599780857,
  0.44961132965460687272,
  0.42755509343028202940,
  0.40524131400498986100,
  0.38268343236508983729,
  0.35989503653498833291,
  0.33688985339222027315,
  0.31368174039889140658,
  0.29028467725446233105,
  0.26671275747489847641,
  0.24298017990326403748,
  0.21910124015687001964,
  0.19509032201612858115,
  0.17096188876030118942,
  0.14673047445536180344,
  0.12241067519921634832,
  0.09801714032956083955,
  0.07356456359966774550,
  0.04906767432741797330,
  0.02454122852291232629,
  0.00000000000000012246,
  -0.02454122852291208343,
  -0.04906767432741772350,
  -0.07356456359966749570,
  -0.09801714032956058975,
  -0.12241067519921609852,
  -0.14673047445536158140,
  -0.17096188876030096737,
  -0.19509032201612835911,
  -0.21910124015686982535,
  -0.24298017990326381543,
  -0.26671275747489825436,
  -0.29028467725446216452,
  -0.31368174039889118454,
  -0.33688985339222010662,
  -0.35989503653498811087,
  -0.38268343236508967076,
  -0.40524131400498969446,
  -0.42755509343028180735,
  -0.44961132965460665067,
  -0.47139673682599764204,
  -0.49289819222978392688,
  -0.51410274419322155026,
  -0.53499761988709693128,
  -0.55557023301960195560,
  -0.57580819141784533866,
  -0.59569930449243324588,
  -0.61523159058062670823,
  -0.63439328416364526575,
  -0.65317284295377653347,
  -0.67155895484701844111,
  -0.68954054473706682948,
  -0.70710678118654746172,
  -0.72424708295146666970,
  -0.74095112535495877282,
  -0.75720884650648423442,
  -0.77301045336273666031,
  -0.78834642762660589455,
  -0.80320753148064505389,
  -0.81758481315158371139,
  -0.83146961230254523567,
  -0.84485356524970711689,
  -0.85772861000027200706,
  -0.87008699110871134952,
  -0.88192126434835493853,
  -0.89322430119551521344,
  -0.90398929312344311615,
  -0.91420975570353046891,
  -0.92387953251128651644,
  -0.93299279883473895669,
  -0.94154406518302080631,
  -0.94952818059303667475,
  -0.95694033573220882438,
  -0.96377606579543984022,
  -0.97003125319454397424,
  -0.97570213003852845901,
  -0.98078528040323031956,
  -0.98527764238894111060,
  -0.98917650996478090342,
  -0.99247953459871007809,
  -0.99518472667219692873,
  -0.99729045667869020697,
  -0.99879545620517240501,
  -0.99969881869620424997,
  -1.00000000000000000000,
  -0.99969881869620424997,
  -0.99879545620517240501,
  -0.99729045667869020697,
  -0.99518472667219692873,
  -0.99247953459871007809,
  -0.98917650996478090342,
  -0.98527764238894122162,
  -0.98078528040323043058,
  -0.97570213003852857003,
  -0.97003125319454397424,
  -0.96377606579543995124,
  -0.95694033573220893540,
  -0.94952818059303678577,
  -0.94154406518302091733,
  -0.93299279883473906771,
  -0.92387953251128662746,
  -0.91420975570353057993,
  -0.90398929312344333820,
  -0.89322430119551532446,
  -0.88192126434835504956,
  -0.87008699110871146054,
  -0.85772861000027222911,
  -0.84485356524970733894,
  -0.83146961230254556874,
  -0.81758481315158404445,
  -0.80320753148064527593,
  -0.78834642762660611659,
  -0.77301045336273688235,
  -0.75720884650648456748,
  -0.74095112535495921691,
  -0.72424708295146711379,
  -0.70710678118654768376,
  -0.68954054473706716255,
  -0.67155895484701866316,
  -0.65317284295377708858,
  -0.63439328416364593188,
  -0.61523159058062737437,
  -0.59569930449243324588,
  -0.57580819141784522763,
  -0.55557023301960217765,
  -0.53499761988709726435,
  -0.51410274419322188333,
  -0.49289819222978425994,
  -0.47139673682599791960,
  -0.44961132965460698374,
  -0.42755509343028252900,
  -0.40524131400499041611,
  -0.38268343236509039240,
  -0.35989503653498799984,
  -0.33688985339221999560,
  -0.31368174039889151761,
  -0.29028467725446244208,
  -0.26671275747489858743,
  -0.24298017990326417626,
  -0.21910124015687015842,
  -0.19509032201612871993,
  -0.17096188876030177228,
  -0.14673047445536238631,
  -0.12241067519921602913,
  -0.09801714032956052036,
  -0.07356456359966742631,
  -0.04906767432741809126,
  -0.02454122852291245119,
};
constexpr inline float dsp_func_log_tbl[] = {
  0.00000000000000000000,
  0.01506374643587593196,
  0.02962246660680085616,
  0.04370892788540595153,
  0.05735280855195209332,
  0.07058107428570727093,
  0.08341829899087943589,
  0.09588693955047750128,
  0.10800757222633085297,
  0.11979909695588887575,
  0.13127891463931895544,
  0.14246308159183262987,
  0.15336644460215734243,
  0.16400275944794229233,
  0.17438479524166411450,
  0.18452442659254400525,
  0.19443271525279304246,
  0.20411998265592479207,
  0.21359587453980230509,
  0.22286941866876369689,
  0.23194907652068211257,
  0.24084278968066460958,
  0.24955802157887665982,
  0.25810179512217035214,
  0.26648072669494099474,
  0.27470105694163204912,
  0.28276867868966498731,
  0.29068916232576230696,
  0.29846777889938080897,
  0.30610952119325612886,
  0.31361912297200161825,
  0.32100107659460114240,
  0.32825964915489175278,
  0.33539889729524374751,
  0.34242268082220622682,
  0.34933467523853622971,
  0.35613838329348096456,
  0.36283714564218327858,
  0.36943415069541385432,
  0.37593244373233097866,
  0.38233493534146151260,
  0.38864440924846876957,
  0.39486352958339993036,
  0.40099484763489801065,
  0.40704080813423670904,
  0.41300375510791348388,
  0.41888593733386791929,
  0.42468951343310817892,
  0.43041655662559524798,
  0.43606905917660687688,
  0.44164893655744347223,
  0.44715803134221915327,
  0.45259811686057549451,
  0.45797090062443668312,
  0.46327802754537295016,
  0.46852108295774486102,
  0.47370159546152518226,
  0.47882103959755678568,
  0.48388083836695899853,
  0.48888236560545267340,
  0.49382694822251510525,
  0.49871586831449610244,
  0.50355036516011286718,
  0.50833163710609519104,
  0.51306084335015578191,
  0.51773910562792635481,
  0.52236750980999746563,
  0.52694710741475025984,
  0.53147891704225502796,
  0.53596392573412554583,
  0.54040309026387023383,
  0.54479733836196242436,
  0.54914756987955049006,
  0.55345465789446091076,
  0.55771944976289367091,
  0.56194276811997989629,
  0.56612541183215570051,
  0.57026815690411103432,
  0.57437175734288836715,
  0.57843694598153816422,
  0.58246443526458147044,
  0.58645491799738624916,
  0.59040906806142601226,
  0.59432754109727170544,
  0.59821097515704590997,
  0.60205999132796228501,
  0.60587519432847769529,
  0.60965717307848377171,
  0.61340650124488704797,
  0.61712373776383822044,
  0.62080942734080135637,
  0.62446410092958160032,
  0.62808827619136520237,
  0.63168245793476318628,
  0.63524713853779657402,
  0.63878279835270224130,
  0.64228990609439484682,
  0.64576891921336831981,
  0.64922028425377953376,
  0.65264443719741593863,
  0.65604180379420784508,
  0.65941279987991441303,
  0.66275783168157398340,
  0.66607729611128130287,
  0.66937158104882221732,
  0.67264106561366954207,
  0.67588612042681661674,
  0.67910710786290129359,
  0.68230438229304901654,
  0.68547829031884177642,
  0.68862917099779996644,
  0.69175735606074240103,
  0.69486317012137543969,
  0.69794693087843984181,
  0.70100894931073109984,
  0.70404952986529367731,
  0.70706897063907259149,
  0.71006756355429323513,
  0.71304559452782867446,
  0.71600334363479911648,
  0.71894108526663880188,
  0.72185908828385125791,
  0.72475761616366773943,
  0.72763692714281102969,
  0.73049727435555744659,
  0.73333890596728124045,
  0.73616206530366035121,
  0.73896699097570728298,
  0.74175391700079051915,
  0.74452307291979491488,
  0.74727468391057372266,
  0.75000897089782603278,
  0.75272615065953907276,
  0.75542643593011971070,
  0.75811003550034161691,
  0.76077715431422088255,
  0.76342799356293722379,
  0.76606275077590457734,
  0.76868161990909711356,
  0.77128479143072647961,
  0.77387245240436863813,
  0.77644478656962645502,
  0.77900197442041996254,
  0.78154419328098290176,
  0.78407161737964969905,
  0.78658441792050637353,
  0.78908276315298142567,
  0.79156681843944620702,
  0.79403674632089293883,
  0.79649270658075532747,
  0.79893485630693561461,
  0.80136334995209634879,
  0.80377833939227660842,
  0.80617997398388707708,
  0.80856840061913792805,
  0.81094376377995092131,
  0.81330620559040545103,
  0.81565586586776461786,
  0.81799288217213050878,
  0.82031738985476820858,
  0.82262952210514572737,
  0.82492940999672603741,
  0.82721718253155518408,
  0.82949296668368066587,
  0.83175688744143860820,
  0.83400906784864448085,
  0.83624962904472133207,
  0.83847869030379784672,
  0.84069636907280886895,
  0.84290278100862703337,
  0.84509804001425670172,
  0.84728225827411807192,
  0.84945554628844666123,
  0.85161801290683902810,
  0.85376976536096482739,
  0.85591090929647428709,
  0.85804154880412220052,
  0.86016178645013263715,
  0.86227172330582513293,
  0.86437145897652578608,
  0.86646109162978246498,
  0.86854071802290333526,
  0.87061043352984157639,
  0.87267033216743983282,
  0.87472050662105760299,
  0.87676104826959444516,
  0.87879204720992920574,
  0.88081359228079114665,
  0.88282577108607651617,
  0.88482867001762732784,
  0.88682237427749022185,
  0.88880696789966262550,
  0.89078253377134730684,
  0.89274915365372464748,
  0.89470690820225606821,
  0.89665587698653370641,
  0.89859613850968400595,
  0.90052777022734309398,
  0.90245084856620882974,
  0.90436544894218606760,
  0.90627164577813279500,
  0.90816951252121858040,
  0.91005912165990565654,
  0.91194054474056207571,
  0.91381385238371659607,
  0.91567911429996395878,
  0.91753639930553110293,
  0.91938577533751097981,
  0.92122730946877362523,
  0.92306106792256348292,
  0.92488711608678786380,
  0.92670551852800731041,
  0.92851633900513619313,
  0.93031964048285398228,
  0.93211548514474507066,
  0.93390393440616303877,
  0.93568504892683512786,
  0.93745888862320592150,
  0.93922551268053289242,
  0.94098497956473414749,
  0.94273734703400002832,
  0.94448267215016856735,
  0.94622101128987556873,
  0.94795242015548120129,
  0.94967695378577987597,
  0.95139466656649851473,
  0.95310561224058765184,
  0.95480984391831114078,
  0.95650741408713824132,
  0.95819837462144330509,
  0.95988277679201750026,
  0.96156067127539601724,
  0.96323210816300752768,
  0.96489713697014556359,
  0.96655580664477158681,
  0.96820816557614863829,
  0.96985426160331089651,
  0.97149414202337613933,
  0.97312785359969866672,
  0.97475544256987067815,
  0.97637695465357565716,
  0.97799243506029320816,
  0.97960192849686322791,
  0.98120547917490841261,
  0.98280313081812264997,
  0.98439492666942374210,
  0.98598090949797689841,
  0.98756112160608933159,
  0.98913560483598073070,
  0.99070440057643105458,
  0.99226754976930919838,
  0.99382509291598364332,
  0.99537707008361930860,
  0.99692352091136227088,
  0.99846448461641412742,
  1.00000000000000000000,
};
constexpr inline float dsp_func_exp_tbl[] = {
  0.00000000000000000000,
  0.00153551538358587258,
  0.00307647908863772912,
  0.00462292991638069140,
  0.00617490708401635668,
  0.00773245023069080162,
  0.00929559942356894542,
  0.01086439516401926930,
  0.01243887839391066841,
  0.01401909050202310159,
  0.01560507333057625790,
  0.01719686918187735003,
  0.01879452082509158739,
  0.02039807150313677209,
  0.02200756493970679184,
  0.02362304534642434284,
  0.02524455743012932185,
  0.02687214640030133328,
  0.02850585797662386067,
  0.03014573839668910349,
  0.03179183442385136171,
  0.03344419335522841319,
  0.03510286302985443641,
  0.03676789183699247232,
  0.03843932872460398276,
  0.04011722320798249974,
  0.04180162537855669491,
  0.04349258591286175868,
  0.04519015608168885922,
  0.04689438775941234816,
  0.04860533343350148527,
  0.05032304621422012403,
  0.05204757984451879871,
  0.05377898871012443127,
  0.05551732784983143265,
  0.05726265296599997168,
  0.05901502043526563046,
  0.06077448731946710758,
  0.06254111137679407850,
  0.06431495107316487214,
  0.06609606559383696123,
  0.06788451485525492934,
  0.06968035951714601772,
  0.07148366099486391789,
  0.07329448147199268959,
  0.07511288391321213620,
  0.07693893207743651708,
  0.07877269053122637477,
  0.08061422466248902019,
  0.08246360069446889707,
  0.08432088570003604122,
  0.08618614761628340393,
  0.08805945525943792429,
  0.08994087834009434346,
  0.09183048747878141960,
  0.09372835422186720500,
  0.09563455105781393240,
  0.09754915143379117026,
  0.09947222977265690602,
  0.10140386149031599405,
  0.10334412301346629359,
  0.10529309179774393179,
  0.10725084634627535252,
  0.10921746622865269316,
  0.11119303210033737450,
  0.11317762572250977815,
  0.11517132998237267216,
  0.11717422891392348383,
  0.11918640771920863131,
  0.12120795279007079426,
  0.12323895173040555484,
  0.12527949337894239701,
  0.12732966783256016718,
  0.12938956647015842361,
  0.13145928197709677576,
  0.13353890837021764604,
  0.13562854102347421392,
  0.13772827669417486707,
  0.13983821354986736285,
  0.14195845119587779948,
  0.14408909070352571291,
  0.14623023463903517261,
  0.14838198709316097190,
  0.15054445371155333877,
  0.15271774172588192808,
  0.15490195998574318725,
  0.15709721899137296663,
  0.15930363092719113105,
  0.16152130969620215328,
  0.16375037095527866793,
  0.16599093215135563018,
  0.16824311255856150282,
  0.17050703331631933413,
  0.17278281746844481592,
  0.17507059000327396259,
  0.17737047789485427263,
  0.17968261014523179142,
  0.18200711782786949122,
  0.18434413413223538214,
  0.18669379440959454897,
  0.18905623622004896767,
  0.19143159938086207195,
  0.19382002601611292292,
  0.19622166060772339158,
  0.19863665004790365121,
  0.20106514369306438539,
  0.20350729341924467253,
  0.20596325367910706117,
  0.20843318156055379298,
  0.21091723684701857433,
  0.21341558207949362647,
  0.21592838262035030095,
  0.21845580671901709824,
  0.22099802557958003746,
  0.22355521343037354498,
  0.22612754759563136187,
  0.22871520856927340937,
  0.23131838009090288644,
  0.23393724922409542266,
  0.23657200643706277621,
  0.23922284568577911745,
  0.24188996449965838309,
  0.24457356406988040032,
  0.24727384934046103826,
  0.24999102910217396722,
  0.25272531608942627734,
  0.25547692708020508512,
  0.25824608299920948085,
  0.26103300902429271702,
  0.26383793469633964879,
  0.26666109403271864853,
  0.26950272564444244239,
  0.27236307285718897031,
  0.27524238383633226057,
  0.27814091171614874209,
  0.28105891473336119812,
  0.28399665636520088352,
  0.28695440547217132554,
  0.28993243644570665385,
  0.29293102936092729749,
  0.29595047013470632269,
  0.29899105068926890016,
  0.30205306912156015819,
  0.30513682987862456031,
  0.30824264393925759897,
  0.31137082900220003356,
  0.31452170968115822358,
  0.31769561770695087244,
  0.32089289213709881743,
  0.32411387957318338326,
  0.32735893438633045793,
  0.33062841895117778268,
  0.33392270388871869713,
  0.33724216831842601660,
  0.34058720012008558697,
  0.34395819620579215492,
  0.34735556280258406137,
  0.35077971574622046624,
  0.35423108078663168019,
  0.35771009390560515318,
  0.36121720164729775870,
  0.36475286146220342598,
  0.36831754206523670270,
  0.37191172380863479763,
  0.37553589907041851070,
  0.37919057265919875466,
  0.38287626223616177956,
  0.38659349875511295203,
  0.39034282692151622829,
  0.39412480567152230471,
  0.39794000867203771499,
  0.40178902484295409003,
  0.40567245890272840558,
  0.40959093193857420978,
  0.41354508200261375084,
  0.41753556473541852956,
  0.42156305401846183578,
  0.42562824265711163285,
  0.42973184309588896568,
  0.43387458816784418847,
  0.43805723188002021473,
  0.44228055023710632909,
  0.44654534210553908924,
  0.45085243012044950994,
  0.45520266163803757564,
  0.45959690973612976617,
  0.46403607426587445417,
  0.46852108295774486102,
  0.47305289258524985119,
  0.47763249019000264539,
  0.48226089437207364519,
  0.48693915664984421809,
  0.49166836289390480896,
  0.49644963483988702180,
  0.50128413168550389756,
  0.50617305177748495026,
  0.51111763439454738212,
  0.51611916163304094596,
  0.52117896040244326983,
  0.52629840453847487325,
  0.53147891704225513898,
  0.53672197245462704984,
  0.54202909937556331688,
  0.54740188313942428344,
  0.55284196865778079122,
  0.55835106344255658328,
  0.56393094082339312312,
  0.56958344337440469651,
  0.57531048656689187659,
  0.58111406266613196969,
  0.58699624489208646061,
  0.59295919186576306892,
  0.59900515236510198935,
  0.60513647041660023618,
  0.61135559075153123043,
  0.61766506465853843189,
  0.62406755626766896583,
  0.63056584930458603466,
  0.63716285435781672142,
  0.64386161670651897992,
  0.65066532476146388131,
  0.65757731917779382869,
  0.66460110270475625249,
  0.67174035084510830274,
  0.67899892340539880209,
  0.68638087702799821521,
  0.69389047880674370461,
  0.70153222110061919103,
  0.70931083767423763753,
  0.71723132131033495718,
  0.72529894305836795088,
  0.73351927330505906077,
  0.74189820487782964786,
  0.75044197842112325691,
  0.75915721031933525165,
  0.76805092347931769314,
  0.77713058133123635862,
  0.78640412546019777817,
  0.79588001734407520793,
  0.80556728474720695754,
  0.81547557340745591148,
  0.82561520475833582999,
  0.83599724055205759665,
  0.84663355539784257431,
  0.85753691840816748115,
  0.86872108536068115558,
  0.88020090304411124915,
  0.89199242777366916091,
  0.90411306044952244321,
  0.91658170100912050859,
  0.92941892571429263192,
  0.94264719144804776096,
  0.95629107211459418725,
  0.97037753339319920975,
  0.98493625356412417560,
  1.00000000000000000000,
};
constexpr inline float dsp_func_perlin_5order_tbl[] = {
  1.00000000000000000000,
  0.99999940045570756553,
  0.99999523189263617162,
  0.99998400267069142977,
  0.99996230495581905906,
  0.99992681405222716418,
  0.99987428773460984566,
  0.99980156558036969994,
  0.99970556830183998631,
  0.99958329707850857027,
  0.99943183288923975738,
  0.99924833584449734847,
  0.99903004451856769474,
  0.99877427528178219784,
  0.99847842163274014293,
  0.99813995353053164283,
  0.99775641672696002704,
  0.99732543209876545198,
  0.99684469497984651287,
  0.99631197449348429807,
  0.99572511288456388989,
  0.99508202485179841901,
  0.99438069687995100931,
  0.99361918657205761107,
  0.99279562198165027809,
  0.99190820094497955672,
  0.99095519041323731901,
  0.98993492578477970678,
  0.98884581023734985372,
  0.98768631406030049646,
  0.98645497398681702972,
  0.98515039252614011733,
  0.98377123729578852540,
  0.98231624035378206639,
  0.98078419753086421018,
  0.97917396776272469516,
  0.97748447242222280540,
  0.97571469465160975965,
  0.97386367869475154446,
  0.97193052922935196936,
  0.96991441069917516682,
  0.96781454664626853646,
  0.96563021904318535604,
  0.96336076762520783667,
  0.96100558922256973382,
  0.95856413709267906942,
  0.95603592025234118701,
  0.95342050280998125178,
  0.95071750329786730571,
  0.94792659400433276762,
  0.94504750030599937727,
  0.94208000000000002849,
  0.93902392263620149127,
  0.93587914884942724481,
  0.93264560969168019966,
  0.92932328596436553081,
  0.92591220755051351077,
  0.92241245274700200962,
  0.91882414759677977223,
  0.91514746522108902926,
  0.91138262515168777522,
  0.90752989266307348970,
  0.90358957810470519334,
  0.89956203623322628093,
  0.89544766554468790964,
  0.89124690760677105494,
  0.88696024639100978781,
  0.88258820760501377478,
  0.87813135802469133306,
  0.87359030482647193061,
  0.86896569491952924125,
  0.86425821427800364471,
  0.85946858727322539284,
  0.85459757600593699856,
  0.84964597963851629103,
  0.84461463372719924880,
  0.83950440955430238876,
  0.83431621346044604337,
  0.82905098617677697170,
  0.82370970215719074847,
  0.81829336891055548531,
  0.81280302633293355363,
  0.80723974603980508391,
  0.80160463069829046567,
  0.79589881335937362472,
  0.79012345679012352395,
  0.78427975280591855078,
  0.77836892160266835106,
  0.77239221108903710622,
  0.76635089621866547827,
  0.76024627832239444203,
  0.75407968444048756318,
  0.74785246665485360928,
  0.74156600142126993802,
  0.73522168890160444210,
  0.72882095229603938158,
  0.72236523717529310673,
  0.71585601081284355729,
  0.70929476151715042942,
  0.70268299796387934109,
  0.69602224852812288880,
  0.68931406061662425788,
  0.68255999999999994454,
  0.67576165014496292205,
  0.66892061154654447463,
  0.66203850106031814082,
  0.65511695123462132528,
  0.64815760964277946421,
  0.64116213821532741512,
  0.63413221257223284510,
  0.62706952135511961899,
  0.61997576555948952226,
  0.61285265786694553825,
  0.60570192197741501428,
  0.59852529194137160662,
  0.59132451149205900176,
  0.58410133337771275031,
  0.57685751869378398826,
  0.56959483621516149299,
  0.56231506172839496038,
  0.55501997736391750493,
  0.54771137092826882586,
  0.54039103523631659698,
  0.53306076744348196428,
  0.52572236837795971454,
  0.51837764187294232965,
  0.51102839409884226463,
  0.50367643289551577990,
  0.49632356710448433113,
  0.48897160590115751333,
  0.48162235812705789240,
  0.47427763162204028546,
  0.46693923255651770265,
  0.45960896476368295893,
  0.45228862907173128516,
  0.44498002263608216200,
  0.43768493827160503962,
  0.43040516378483839599,
  0.42314248130621634481,
  0.41589866662228747174,
  0.40867548850794088722,
  0.40147470805862850440,
  0.39429807802258509675,
  0.38714734213305446175,
  0.38002423444051069978,
  0.37293047864488015897,
  0.36586778742776715490,
  0.35883786178467280692,
  0.35184239035722031375,
  0.34488304876537845267,
  0.33796149893968197020,
  0.33107938845345552537,
  0.32423834985503674488,
  0.31744000000000038852,
  0.31068593938337540905,
  0.30397775147187733324,
  0.29731700203612065891,
  0.29070523848284901547,
  0.28414398918715644271,
  0.27763476282470733736,
  0.27117904770396106251,
  0.26477831109839544688,
  0.25843399857873006198,
  0.25214753334514639072,
  0.24592031555951265887,
  0.23975372167760578002,
  0.23364910378133441071,
  0.22760778891096267174,
  0.22163107839733209303,
  0.21572024719408133819,
  0.20987654320987703116,
  0.20410118664062615323,
  0.19839536930170931228,
  0.19276025396019491609,
  0.18719697366706666841,
  0.18170663108944395958,
  0.17629029784280891846,
  0.17094901382322325034,
  0.16568378653955395663,
  0.16049559044569772226,
  0.15538536627280041813,
  0.15035402036148282079,
  0.14540242399406322349,
  0.14053141272677471818,
  0.13574178572199535608,
  0.13103430508047075875,
  0.12640969517352784734,
  0.12186864197530900000,
  0.11741179239498666931,
  0.11303975360899043423,
  0.10875309239322916710,
  0.10455233445531186831,
  0.10043796376677383009,
  0.09641042189529525075,
  0.09247010733692784257,
  0.08861737484831255784,
  0.08485253477891019358,
  0.08117585240322000573,
  0.07758754725299743527,
  0.07408779244948693332,
  0.07067671403563346999,
  0.06735439030831891216,
  0.06412085115057220008,
  0.06097607736379995202,
  0.05791999999999930537,
  0.05495249969400095580,
  0.05207340599566823158,
  0.04928249670213347144,
  0.04657949719001930333,
  0.04396407974765903504,
  0.04143586290732148569,
  0.03899441077743048822,
  0.03663923237479238537,
  0.03436978095681553214,
  0.03218545335373157457,
  0.03008558930082383398,
  0.02806947077064858576,
  0.02613632130524745634,
  0.02428530534838913013,
  0.02251552757777819380,
  0.02082603223727641506,
  0.01921580246913556778,
  0.01768375964621959895,
  0.01622876270421169664,
  0.01484960747385954960,
  0.01354502601318152699,
  0.01231368593970039171,
  0.01115418976264859197,
  0.01006507421522151446,
  0.00904480958676234792,
  0.00809179905502155350,
  0.00720437801835061009,
  0.00638081342794194484,
  0.00561930312004932375,
  0.00491797514820113690,
  0.00427488711543499988,
  0.00368802550651636807,
  0.00315530502015182179,
  0.00267456790123432597,
  0.00224358327303963989,
  0.00186004646946713592,
  0.00152157836726019013,
  0.00122572471821946749,
  0.00096995548143219423,
  0.00075166415550231847,
  0.00056816711076024262,
  0.00041670292149120769,
  0.00029443169816012471,
  0.00019843441963018904,
  0.00012571226539037639,
  0.00007318594777316889,
  0.00003769504418116298,
  0.00001599732930834818,
  0.00000476810736316224,
  0.00000059954429332265,
  0.00000000000000000000,
};