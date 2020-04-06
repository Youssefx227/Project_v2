
 #lancement de n flux de SV_publisher simultanément 
  
 #                            exécutable     sv_sub      appid_sv durée souscription [min]
  sudo gnome-terminal   -x  ./sv_subscriber   lo 0x4000  "./files/logfiles/time_file.csv" 1
  #sudo gnome-terminal   -x  ./sv_subscriber eno1       "./test_bool2.csv"   &
  #sudo gnome-terminal   -x  ./sv_subscriber  eno1      "./test_natif/subscribe5/time_stamp3.csv" &
  #sudo gnome-terminal   -x  ./sv_subscriber  eno1      "./test_natif/subscribe5/time_stamp4.csv" &
  #sudo gnome-terminal   -x  ./sv_subscriber  eno1      "./test_natif/subscribe5/time_stamp5.csv"&

