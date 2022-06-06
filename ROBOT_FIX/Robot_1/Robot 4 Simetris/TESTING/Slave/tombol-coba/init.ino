void init_TombolMenu(int Tombol[]){
  int size = sizeof(Tombol) / sizeof(int);
  
  for (int i = 0; i < size; i++) {
    pinMode(Tombol[i], INPUT_PULLUP);
  }
}
