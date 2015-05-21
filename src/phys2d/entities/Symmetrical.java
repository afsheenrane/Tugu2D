package phys2d.entities;

/**
 * This interface is to be implemented in shapes which have some form on symmetry.
 * This will greatly reduce the runtime of the SAT collision detection algorithm. 
 * @author Afsheen
 *
 */
public interface Symmetrical {
	/**
	 * Return only the surface normals of a shape that lie on different axes.
	 * @return
	 */
	public Vec2D[] getUniqueNormals();
}
