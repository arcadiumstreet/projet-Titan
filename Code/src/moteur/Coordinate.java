package moteur;

public class Coordinate {
	double x;//l'abscisse
	double y;//l'ordonnée
	
	/**
	 * 
	 * @return la vauleur de x, l'abscisse de la coordonnée representée
	 */
	public double getX() {
		return x;
	}

	/**	 
	 * donne à l'abscisse la valeur de y
	 * 
	 * @param x l'abscisse en reel
	 */
	public void setX(double x) {
		this.x = x;
	}

	/**
	 * 
	 * @return la vauleur de y, l'ordonnée de la coordonnée representée
	 */
	public double getY() {
		return y;
	}

	
	/**
	 * donne à l'ordonnée la valeur de y
	 * 
	 * @param y l'ordonnée en reel
	 */
	public void setY(double y) {
		this.y = y;
	}

	/**
	 * crée une coordonnée d'abscisse x et d'ordonnée y
	 * 
	 * @param x valeur de l'abscisse en reel
	 * @param y valeur de l'ordonnée en reel
	 */
	public Coordinate(double x, double y) {
		this.x = x;
		this.y = y;
	}

}
