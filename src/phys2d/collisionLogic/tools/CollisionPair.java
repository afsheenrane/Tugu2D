package phys2d.collisionLogic.tools;

import phys2d.entities.shapes.Shape;

/**
 * A small class to hold 2 pairs of shapes that have been checked for collision. <br>
 * Is more or less being used so that it's hashing can be used to make the the
 * check that prevents redundant collision checks, quicker.
 * 
 * @author Afsheen
 *
 */
public class CollisionPair {

    private final Shape s1, s2;

    public CollisionPair(Shape s1, Shape s2) {
        this.s1 = s1;
        this.s2 = s2;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * (result + ((s1 == null) ? 0 : s1.hashCode()) + ((s2 == null) ? 0 : s2.hashCode()));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;

        if (getClass() != obj.getClass())
            return false;

        CollisionPair other = (CollisionPair) obj;

        if (this.hashCode() == other.hashCode())
            return true;

        return false;
    }

}
