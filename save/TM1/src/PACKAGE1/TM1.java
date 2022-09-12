package PACKAGE1;

public class TM1 {
	int denominateur ;
	int numerateur ;
	
	public TM1() {
		this (0,1);
	}
	public TM1(int x) {
		this(x,1);
	}
	public TM1(int n,int d) {
		if (d==0) {
			throw new IllegalArgumentException("denominateur est égale a 0") ;}
	denominateur =d;
	numerateur = n;
}
	public TM1(String ch) {
		int r = ch.indexOf("/");
		numerateur =Integer.parseInt(ch.substring(0,r));
		denominateur =Integer.parseInt(ch.substring(r+1, ch.length()));
	}
	
	public int getDenominateur() {
		return denominateur;
	}
	public void setDenominateur(int denominateur) {
		this.denominateur = denominateur;
	}
	public int getNumerateur() {
		return numerateur;
	}
	public void setNumerateur(int numerateur) {
		this.numerateur = numerateur;
	}
	public String toString() {
		return numerateur +" / "+denominateur ;
	}
//	public static Fraction valueOf(String ch) {
//		Fraction c = new Fraction(ch);
//		return c ;}
	
	public int compareTo(TM1 f) {
	
		if (this.numerateur<f.numerateur ) {
			return -1;}
		else if (this.numerateur<f.numerateur) {
			return 1 ;}
		else if(this.numerateur<f.numerateur) {
			return 0;}
		else {return 3;}
	}
	public boolean equals(Object obj) {
		return(obj instanceof TM1)&& (compareTo((TM1)obj)==0);
		 
	}
	public static int pgcd (int a,int b) {
		int r = a%b;
		if (r== 0 ) {return b;}
		return pgcd(b,r);
	}
	
	public static int ppcm (int a ,int b) {
		return (a*b)/pgcd(a,b) ;
	}
	
	public TM1 plus(TM1 f) {
		int p = ppcm(denominateur,f.denominateur);
		int n = numerateur* p/denominateur-((f.numerateur*p)/f.denominateur); 
		return new TM1(n,p);
	}
	
	public TM1 moins(TM1 f) {
		int n=this.numerateur-f.numerateur ;
		int d=this.denominateur-f.denominateur ;
		TM1 s = new TM1(n,d);
		return s;
	}
	public TM1 fois(TM1 f) {
		int n=this.numerateur*f.numerateur ;
		int d=this.denominateur*f.denominateur ;
		TM1 s = new TM1(n,d);
		return s;
	}
	public TM1 diviseepar(TM1 f) {
		int a=this.numerateur*f.denominateur;
		int b=this.denominateur*f.numerateur;
		TM1 s = new TM1(a,b);
		return s;
	}
	
	public static TM1 exparith(String d) {
		int i = 0 ;
		int z =0;
		TM1 [] tabf = new TM1[10];
		char [] tabs = new char[10];
		String f = "";
		while(i<d.length()){
			if(Character.isDigit(d.charAt(i))&& d.charAt(i)=='/'){
			f = f+'v';
			}
			else {z++;
				tabf[z]=new TM1(f);
				tabs[i] = d.charAt(i);
				f = "";
			}
			d=d.substring(1);
		i++;}
		int t = tabs.length ;
		int r =1;
		while (t>r) {
			if (tabs[r]=='+') {
				tabf[0].plus(tabf[r]);
			}
			if (tabs[r]=='-') {
				tabf[0].moins(tabf[r]);
			}
			if (tabs[r]=='*') {
				tabf[0].fois(tabf[r]);
			}
			if (tabs[r]=='/') {
				tabf[0].diviseepar(tabf[r]);
			}	
		r++;}
	return tabf[0];}
	
	public static void main(String[] args) {
		TM1 f= new TM1("45/65");
		
		System.out.println(f.fois(new TM1("45/68")));
	}

}
