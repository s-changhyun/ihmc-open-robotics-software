package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;

import us.ihmc.utilities.exeptions.NoConvergenceException;
import cern.colt.Arrays;

public class ContactPointWrenchOptimizerNative
{
   public static final int NUMBER_OF_POINTS_PER_CONTACT = 4;
   public static final int NUMBER_OF_SUPPORT_VECTORS = 4;
   public static final int MAX_NUMBER_OF_CONTACTS = 4;
   public static final int WRENCH_LENGTH = 6;

   private static native void initialize();

   private static native ByteBuffer getABuffer();

   private static native ByteBuffer getWBuffer();

   private static native ByteBuffer getCBuffer();
   
   private static native ByteBuffer getRhoMinBuffer();

   private static native ByteBuffer getRhoBuffer();

   private static native int solveNative(double epsilon);

   private static native double getOptValNative();

   private static final Object solveConch = new Object();

   private static final DoubleBuffer aDoubleBuffer;
   private static final DoubleBuffer cDoubleBuffer;
   private static final DoubleBuffer wDoubleBuffer;
   private static final DoubleBuffer rhoMinDoubleBuffer;
   private static final DoubleBuffer rhoDoubleBuffer;

   public static DoubleBuffer setupBuffer(ByteBuffer buffer)
   {
      buffer.order(ByteOrder.nativeOrder());

      return buffer.asDoubleBuffer();
   }

   public static void setBufferToArray(DoubleBuffer buffer, double[] array)
   {
      buffer.rewind();
      buffer.put(array);
   }

   static
   {
      System.loadLibrary("ContactPointWrenchOptimizer");

      initialize();

      aDoubleBuffer = setupBuffer(getABuffer());
      cDoubleBuffer = setupBuffer(getCBuffer());
      wDoubleBuffer = setupBuffer(getWBuffer());
      rhoMinDoubleBuffer = setupBuffer(getRhoMinBuffer());
      rhoDoubleBuffer = setupBuffer(getRhoBuffer());
   }

   private final double[] rho = new double[NUMBER_OF_POINTS_PER_CONTACT * NUMBER_OF_SUPPORT_VECTORS * MAX_NUMBER_OF_CONTACTS];
   private double optval = 0.0;

   public int solve(double[] a, double[] c, double[] w, double[] rhoMin, double epsilon) throws NoConvergenceException
   {
      int numberOfIterations;

      synchronized (solveConch)
      {
         setBufferToArray(aDoubleBuffer, a);
         setBufferToArray(cDoubleBuffer, c);
         setBufferToArray(wDoubleBuffer, w);
         setBufferToArray(rhoMinDoubleBuffer, rhoMin);

         numberOfIterations = solveNative(epsilon);

         rhoDoubleBuffer.rewind();
         rhoDoubleBuffer.get(rho);

         optval = getOptValNative();
      }

      if (numberOfIterations < 0)
      {
         throw new NoConvergenceException();
      }

      return numberOfIterations;

   }

   public double[] getRho()
   {
      return rho;
   }

   public double getOptval()
   {
      return optval;
   }

   public static void main(String[] args) throws NoConvergenceException
   {
      double[] a = new double[WRENCH_LENGTH * NUMBER_OF_SUPPORT_VECTORS * NUMBER_OF_POINTS_PER_CONTACT * MAX_NUMBER_OF_CONTACTS];
      double[] w = new double[WRENCH_LENGTH];
      double[] c = new double[WRENCH_LENGTH * WRENCH_LENGTH];
      double[] rhoMin = new double[NUMBER_OF_POINTS_PER_CONTACT * NUMBER_OF_SUPPORT_VECTORS * MAX_NUMBER_OF_CONTACTS];
      double[] epsilon = new double[1];

      load_default_data(a, w, c, rhoMin, epsilon);

      ContactPointWrenchOptimizerNative contactPointWrenchOptimizerNative = new ContactPointWrenchOptimizerNative();

      long time = System.nanoTime();
      for (int i = 0; true; i++)
      {
         if (i % 10000 == 0)
         {
            System.out.println("10000 iterations took " + (System.nanoTime() - time) / 1e9 + " seconds");

            System.out.println(Arrays.toString(contactPointWrenchOptimizerNative.getRho()));

            time = System.nanoTime();
         }

         contactPointWrenchOptimizerNative.solve(a, c, w, rhoMin, epsilon[0]);
      }
   }

   private static void load_default_data(double[] A, double[] W, double[] C, double[] rhomin, double[] epsilon)
   {
      A[0] = 0.20319161029830202;
      A[1] = 0.8325912904724193;
      A[2] = -0.8363810443482227;
      A[3] = 0.04331042079065206;
      A[4] = 1.5717878173906188;
      A[5] = 1.5851723557337523;
      A[6] = -1.497658758144655;
      A[7] = -1.171028487447253;
      A[8] = -1.7941311867966805;
      A[9] = -0.23676062539745413;
      A[10] = -1.8804951564857322;
      A[11] = -0.17266710242115568;
      A[12] = 0.596576190459043;
      A[13] = -0.8860508694080989;
      A[14] = 0.7050196079205251;
      A[15] = 0.3634512696654033;
      A[16] = -1.9040724704913385;
      A[17] = 0.23541635196352795;
      A[18] = -0.9629902123701384;
      A[19] = -0.3395952119597214;
      A[20] = -0.865899672914725;
      A[21] = 0.7725516732519853;
      A[22] = -0.23818512931704205;
      A[23] = -1.372529046100147;
      A[24] = 0.17859607212737894;
      A[25] = 1.1212590580454682;
      A[26] = -0.774545870495281;
      A[27] = -1.1121684642712744;
      A[28] = -0.44811496977740495;
      A[29] = 1.7455345994417217;
      A[30] = 1.9039816898917352;
      A[31] = 0.6895347036512547;
      A[32] = 1.6113364341535923;
      A[33] = 1.383003485172717;
      A[34] = -0.48802383468444344;
      A[35] = -1.631131964513103;
      A[36] = 0.6136436100941447;
      A[37] = 0.2313630495538037;
      A[38] = -0.5537409477496875;
      A[39] = -1.0997819806406723;
      A[40] = -0.3739203344950055;
      A[41] = -0.12423900520332376;
      A[42] = -0.923057686995755;
      A[43] = -0.8328289030982696;
      A[44] = -0.16925440270808823;
      A[45] = 1.442135651787706;
      A[46] = 0.34501161787128565;
      A[47] = -0.8660485502711608;
      A[48] = -0.8880899735055947;
      A[49] = -0.1815116979122129;
      A[50] = -1.17835862158005;
      A[51] = -1.1944851558277074;
      A[52] = 0.05614023926976763;
      A[53] = -1.6510825248767813;
      A[54] = -0.06565787059365391;
      A[55] = -0.5512951504486665;
      A[56] = 0.8307464872626844;
      A[57] = 0.9869848924080182;
      A[58] = 0.7643716874230573;
      A[59] = 0.7567216550196565;
      A[60] = -0.5055995034042868;
      A[61] = 0.6725392189410702;
      A[62] = -0.6406053441727284;
      A[63] = 0.29117547947550015;
      A[64] = -0.6967713677405021;
      A[65] = -0.21941980294587182;
      A[66] = -1.753884276680243;
      A[67] = -1.0292983112626475;
      A[68] = 1.8864104246942706;
      A[69] = -1.077663182579704;
      A[70] = 0.7659100437893209;
      A[71] = 0.6019074328549583;
      A[72] = 0.8957565577499285;
      A[73] = -0.09964555746227477;
      A[74] = 0.38665509840745127;
      A[75] = -1.7321223042686946;
      A[76] = -1.7097514487110663;
      A[77] = -1.2040958948116867;
      A[78] = -1.3925560119658358;
      A[79] = -1.5995826216742213;
      A[80] = -1.4828245415645833;
      A[81] = 0.21311092723061398;
      A[82] = -1.248740700304487;
      A[83] = 1.808404972124833;
      A[84] = 0.7264471152297065;
      A[85] = 0.16407869343908477;
      A[86] = 0.8287224032315907;
      A[87] = -0.9444533161899464;
      A[88] = 1.7069027370149112;
      A[89] = 1.3567722311998827;
      A[90] = 0.9052779937121489;
      A[91] = -0.07904017565835986;
      A[92] = 1.3684127435065871;
      A[93] = 0.979009293697437;
      A[94] = 0.6413036255984501;
      A[95] = 1.6559010680237511;
      A[96] = 0.5346622551502991;
      A[97] = -0.5362376605895625;
      A[98] = 0.2113782926017822;
      A[99] = -1.2144776931994525;
      A[100] = -1.2317108144255875;
      A[101] = 0.9026784957312834;
      A[102] = 1.1397468137245244;
      A[103] = 1.8883934547350631;
      A[104] = 1.4038856681660068;
      A[105] = 0.17437730638329096;
      A[106] = -1.6408365219077408;
      A[107] = -0.04450702153554875;
      A[108] = 1.7117453902485025;
      A[109] = 1.1504727980139053;
      A[110] = -0.05962309578364744;
      A[111] = -0.1788825540764547;
      A[112] = -1.1280569263625857;
      A[113] = -1.2911464767927057;
      A[114] = -1.7055053231225696;
      A[115] = 1.56957275034837;
      A[116] = 0.5607064675962357;
      A[117] = -1.4266707301147146;
      A[118] = -0.3434923211351708;
      A[119] = -1.8035643024085055;
      A[120] = -1.1625066019105454;
      A[121] = 0.9228324965161532;
      A[122] = 0.6044910817663975;
      A[123] = -0.0840868104920891;
      A[124] = -0.900877978017443;
      A[125] = 0.608892500264739;
      A[126] = 1.8257980452695217;
      A[127] = -0.25791777529922877;
      A[128] = -1.7194699796493191;
      A[129] = -1.7690740487081298;
      A[130] = -1.6685159248097703;
      A[131] = 1.8388287490128845;
      A[132] = 0.16304334474597537;
      A[133] = 1.3498497306788897;
      A[134] = -1.3198658230514613;
      A[135] = -0.9586197090843394;
      A[136] = 0.7679100474913709;
      A[137] = 1.5822813125679343;
      A[138] = -0.6372460621593619;
      A[139] = -1.741307208038867;
      A[140] = 1.456478677642575;
      A[141] = -0.8365102166820959;
      A[142] = 0.9643296255982503;
      A[143] = -1.367865381194024;
      A[144] = 0.7798537405635035;
      A[145] = 1.3656784761245926;
      A[146] = 0.9086083149868371;
      A[147] = -0.5635699005460344;
      A[148] = 0.9067590059607915;
      A[149] = -1.4421315032701587;
      A[150] = -0.7447235390671119;
      A[151] = -0.32166897326822186;
      A[152] = 1.5088481557772684;
      A[153] = -1.385039165715428;
      A[154] = 1.5204991609972622;
      A[155] = 1.1958572768832156;
      A[156] = 1.8864971883119228;
      A[157] = -0.5291880667861584;
      A[158] = -1.1802409243688836;
      A[159] = -1.037718718661604;
      A[160] = 1.3114512056856835;
      A[161] = 1.8609125943756615;
      A[162] = 0.7952399935216938;
      A[163] = -0.07001183290468038;
      A[164] = -0.8518009412754686;
      A[165] = 1.3347515373726386;
      A[166] = 1.4887180335977037;
      A[167] = -1.6314736327976336;
      A[168] = -1.1362021159208933;
      A[169] = 1.327044361831466;
      A[170] = 1.3932155883179842;
      A[171] = -0.7413880049440107;
      A[172] = -0.8828216126125747;
      A[173] = -0.27673991192616;
      A[174] = 0.15778600105866714;
      A[175] = -1.6177327399735457;
      A[176] = 1.3476485548544606;
      A[177] = 0.13893948140528378;
      A[178] = 1.0998712601636944;
      A[179] = -1.0766549376946926;
      A[180] = 1.8611734044254629;
      A[181] = 1.0041092292735172;
      A[182] = -0.6276245424321543;
      A[183] = 1.794110587839819;
      A[184] = 0.8020471158650913;
      A[185] = 1.362244341944948;
      A[186] = -1.8180107765765245;
      A[187] = -1.7774338357932473;
      A[188] = 0.9709490941985153;
      A[189] = -0.7812542682064318;
      A[190] = 0.0671374633729811;
      A[191] = -1.374950305314906;
      A[192] = 1.9118096386279388;
      A[193] = 0.011004190697677885;
      A[194] = 1.3160043138989015;
      A[195] = -1.7038488148800144;
      A[196] = -0.08433819112864738;
      A[197] = -1.7508820783768964;
      A[198] = 1.536965724350949;
      A[199] = -0.21675928514816478;
      A[200] = -1.725800326952653;
      A[201] = -1.6940148707361717;
      A[202] = 0.15517063201268;
      A[203] = -1.697734381979077;
      A[204] = -1.264910727950229;
      A[205] = -0.2545716633339441;
      A[206] = -0.008868675926170244;
      A[207] = 0.3332476609670296;
      A[208] = 0.48205072561962936;
      A[209] = -0.5087540014293261;
      A[210] = 0.4749463319223195;
      A[211] = -1.371021366459455;
      A[212] = -0.8979660982652256;
      A[213] = 1.194873082385242;
      A[214] = -1.3876427970939353;
      A[215] = -1.106708108457053;
      A[216] = -1.0280872812241797;
      A[217] = -0.08197078070773234;
      A[218] = -1.9970179118324083;
      A[219] = -1.878754557910134;
      A[220] = -0.15380739340877803;
      A[221] = -1.349917260533923;
      A[222] = 0.7180072150931407;
      A[223] = 1.1808183487065538;
      A[224] = 0.31265343495084075;
      A[225] = 0.7790599086928229;
      A[226] = -0.4361679370644853;
      A[227] = -1.8148151880282066;
      A[228] = -0.24231386948140266;
      A[229] = -0.5120787511622411;
      A[230] = 0.3880129688013203;
      A[231] = -1.4631273212038676;
      A[232] = -1.0891484131126563;
      A[233] = 1.2591296661091191;
      A[234] = -0.9426978934391474;
      A[235] = -0.358719180371347;
      A[236] = 1.7438887059831263;
      A[237] = -0.8977901479165817;
      A[238] = -1.4188401645857445;
      A[239] = 0.8080805173258092;
      A[240] = 0.2682662017650985;
      A[241] = 0.44637534218638786;
      A[242] = -1.8318765960257055;
      A[243] = -0.3309324209710929;
      A[244] = -1.9829342633313622;
      A[245] = -1.013858124556442;
      A[246] = 0.8242247343360254;
      A[247] = -1.753837136317201;
      A[248] = -0.8212260055868805;
      A[249] = 1.9524510112487126;
      A[250] = 1.884888920907902;
      A[251] = -0.0726144452811801;
      A[252] = 0.9427735461129836;
      A[253] = 0.5306230967445558;
      A[254] = -0.1372277142250531;
      A[255] = 1.4282657305652786;
      A[256] = -1.309926991335284;
      A[257] = 1.3137276889764422;
      A[258] = -1.8317219061667278;
      A[259] = 1.4678147672511939;
      A[260] = 0.703986349872991;
      A[261] = -0.2163435603565258;
      A[262] = 0.6862809905371079;
      A[263] = -0.15852598444303245;
      A[264] = 1.1200128895143409;
      A[265] = -1.5462236645435308;
      A[266] = 0.0326297153944215;
      A[267] = 1.4859581597754916;
      A[268] = 1.71011710324809;
      A[269] = -1.1186546738067493;
      A[270] = -0.9922787897815244;
      A[271] = 1.6160498864359547;
      A[272] = -0.6179306451394861;
      A[273] = -1.7725097038051376;
      A[274] = 0.8595466884481313;
      A[275] = -0.3423245633865686;
      A[276] = 0.9412967499805762;
      A[277] = -0.09163346622652258;
      A[278] = 0.002262217745727657;
      A[279] = -0.3297523583656421;
      A[280] = -0.8380604158593941;
      A[281] = 1.6028434695494038;
      A[282] = 0.675150311940429;
      A[283] = 1.1553293733718686;
      A[284] = 1.5829581243724693;
      A[285] = -0.9992442304425597;
      A[286] = 1.6792824558896897;
      A[287] = 1.4504203490342324;
      A[288] = 0.02434104849994556;
      A[289] = 0.27160869657612263;
      A[290] = -1.5402710478528858;
      A[291] = 1.0484633622310744;
      A[292] = -1.3070999712627054;
      A[293] = 0.13534416402363814;
      A[294] = -1.4942507790851232;
      A[295] = -1.708331625671371;
      A[296] = 0.436109775042258;
      A[297] = -0.03518748153727991;
      A[298] = 0.6992397389570906;
      A[299] = 1.1634167322171374;
      A[300] = 1.9307499705822648;
      A[301] = -1.6636772756932747;
      A[302] = 0.5248484497343218;
      A[303] = 0.30789958152579144;
      A[304] = 0.602568707166812;
      A[305] = 0.17271781925751872;
      A[306] = 0.2294695501208066;
      A[307] = 1.4742185345619543;
      A[308] = -0.1919535345136989;
      A[309] = 0.13990231452144553;
      A[310] = 0.7638548150610602;
      A[311] = -1.6420200344195646;
      A[312] = -0.27229872445076087;
      A[313] = -1.5914631171820468;
      A[314] = -1.4487604283558668;
      A[315] = -1.991497766136364;
      A[316] = -1.1611742553535152;
      A[317] = -1.133450950247063;
      A[318] = 0.06497792493777155;
      A[319] = 0.28083295396097263;
      A[320] = 1.2958447220129887;
      A[321] = -0.05315524470737154;
      A[322] = 1.5658183956871667;
      A[323] = -0.41975684089933685;
      A[324] = 0.97844578833777;
      A[325] = 0.2110290496695293;
      A[326] = 0.4953003430893044;
      A[327] = -0.9184320124667495;
      A[328] = 1.750380031759156;
      A[329] = 1.0786188614315915;
      A[330] = -1.4176198837203735;
      A[331] = 0.149737479778294;
      A[332] = 1.9831452222223418;
      A[333] = -1.8037746699794734;
      A[334] = -0.7887206483295461;
      A[335] = 0.9632534854086652;
      A[336] = -1.8425542093895406;
      A[337] = 0.986684363969033;
      A[338] = 0.2936851199350441;
      A[339] = 0.9268227022482662;
      A[340] = 0.20333038350653299;
      A[341] = 1.7576139132046351;
      A[342] = -0.614393188398918;
      A[343] = 0.297877839744912;
      A[344] = -1.796880083990895;
      A[345] = 0.21373133661742738;
      A[346] = -0.32242822540825156;
      A[347] = 1.9326471511608059;
      A[348] = 1.7824292753481785;
      A[349] = -1.4468823405675986;
      A[350] = -1.8335374338761512;
      A[351] = -1.5172997317243713;
      A[352] = -1.229012129120719;
      A[353] = 0.9046719772422094;
      A[354] = 0.17591181415489432;
      A[355] = 0.13970133814112584;
      A[356] = -0.14185208214985234;
      A[357] = -1.9732231264739348;
      A[358] = -0.4301123458221334;
      A[359] = 1.9957537650387742;
      A[360] = 1.2811648216477893;
      A[361] = 0.2914428437588219;
      A[362] = -1.214148157218884;
      A[363] = 1.6818776980374155;
      A[364] = -0.30341101038214635;
      A[365] = 0.47730909231793106;
      A[366] = -1.187569373035299;
      A[367] = -0.6877370247915531;
      A[368] = -0.6201861482616171;
      A[369] = -0.4209925183921568;
      A[370] = -1.9110724537712471;
      A[371] = 0.6413882087807936;
      A[372] = -1.3200399280087032;
      A[373] = 0.41320105301312626;
      A[374] = 0.4783213861392275;
      A[375] = 0.7916189857293743;
      A[376] = -0.8322752558146558;
      A[377] = -0.8318720537426154;
      A[378] = 1.0221179076113445;
      A[379] = -0.4471032189262627;
      A[380] = -1.3901469561676985;
      A[381] = 1.6210596051208572;
      A[382] = -1.9476687601912737;
      A[383] = 1.5459376306231292;
      W[0] = -0.830972896191656;
      W[1] = -0.47269983955176276;
      W[2] = 1.913620609584223;
      W[3] = -0.25329703423935124;
      W[4] = 0.8635279149674653;
      W[5] = -0.35046893227111564;
      C[0] = 1.9135358121693091;
      C[1] = 1.7194904992103375;
      C[2] = 1.4806917884353878;
      C[3] = 1.08422164898412;
      C[4] = 1.3636338678287099;
      C[5] = 1.49060670234726;
      epsilon[0] = 0.5677283669027675;
      rhomin[0] = 1.0692810188392967;
      rhomin[1] = 0.4193021363633158;
      rhomin[2] = 0.9886591510835825;
      rhomin[3] = 1.0560103903142182;
      rhomin[4] = 1.346719281208232;
      rhomin[5] = 1.4907316901639895;
      rhomin[6] = 1.4599474840511448;
      rhomin[7] = 0.8482318005770975;
      rhomin[8] = 0.9119046622137899;
      rhomin[9] = 1.7470142029395843;
      rhomin[10] = 0.7255758451412804;
      rhomin[11] = 1.4760656619152708;
      rhomin[12] = 1.9881344633800206;
      rhomin[13] = 1.849616767073924;
      rhomin[14] = 1.098473735584856;
      rhomin[15] = 0.610222773749272;
      rhomin[16] = 1.2446252717017003;
      rhomin[17] = 1.3686033364624297;
      rhomin[18] = 1.0539245098325878;
      rhomin[19] = 0.6829532616466891;
      rhomin[20] = 0.9108531426787896;
      rhomin[21] = 0.16358148603036082;
      rhomin[22] = 0.5825644099978542;
      rhomin[23] = 0.28979350997045517;
      rhomin[24] = 1.3329614616429688;
      rhomin[25] = 1.9184682830766584;
      rhomin[26] = 0.314469366131227;
      rhomin[27] = 0.05658814370325427;
      rhomin[28] = 1.4827143384325552;
      rhomin[29] = 0.7083289795353997;
      rhomin[30] = 1.0119325532686425;
      rhomin[31] = 0.12209615035708277;
      rhomin[32] = 0.35552989347622943;
      rhomin[33] = 1.3910125838816303;
      rhomin[34] = 1.2104212392344114;
      rhomin[35] = 1.706822444837799;
      rhomin[36] = 1.9258464270765379;
      rhomin[37] = 0.719230198210479;
      rhomin[38] = 1.2404970133216588;
      rhomin[39] = 0.8953548244265135;
      rhomin[40] = 1.0111939253992013;
      rhomin[41] = 0.7830035171794212;
      rhomin[42] = 1.9547884538972506;
      rhomin[43] = 1.2472756349168423;
      rhomin[44] = 0.2837708549853222;
      rhomin[45] = 1.395456882873338;
      rhomin[46] = 1.9315125146691867;
      rhomin[47] = 1.7896987733060534;
      rhomin[48] = 1.1160081667356323;
      rhomin[49] = 0.029429567497201603;
      rhomin[50] = 1.611092663536274;
      rhomin[51] = 1.8637226800022804;
      rhomin[52] = 1.4678579640832892;
      rhomin[53] = 0.8579062545834188;
      rhomin[54] = 0.7616822167723687;
      rhomin[55] = 1.4892095273100956;
      rhomin[56] = 0.21570219429972615;
      rhomin[57] = 1.5693916945518;
      rhomin[58] = 0.997610436759998;
      rhomin[59] = 0.14023802625372928;
      rhomin[60] = 1.6460904282573636;
      rhomin[61] = 0.783414954640167;
      rhomin[62] = 0.21352987136032153;
      rhomin[63] = 0.3475968884162506;
   }
}
