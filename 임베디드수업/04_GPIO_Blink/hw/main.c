// #define P0HW_ADDR 0x400000000 //hardware 영역
// #define P0HW (*(unsigned char*)P0HW_ADDR) // point access
int FACT(int a){
	if(a==1){
	return 1;
	}
	return a*FACT(a-1);
}
int main(){
	char P0;

	//clear
	P0 = 0x95; //hex code 1001 0101(149) >> 1000 0101 (133)
	P0 &= ~(0x10); // 0001 0000 regrassion >> 1110 1111 
	P0 -= (0X10);
	P0 /= 2;

	P0=FACT(5);

	while(1);
}
