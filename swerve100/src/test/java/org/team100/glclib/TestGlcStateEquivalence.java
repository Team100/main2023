package org.team100.glclib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.glclib.GlcMath;

public class TestGlcStateEquivalence {



/**
 * \brief Test that the square function squares a number.
 */
@Test
public void testOrder(){
  double x = 2.0;
  
  assertEquals(GlcMath.sqr(x),4.0);
}

/**
 * \brief Tests that the size of input and output to vecFloor is the same, and that the element-wise floor operation works as expected.
 */
@Test
public void testFloor(){
  double[]x = {1.1,2.2,3.99,4.05,-2.01};
  int[] y = GlcMath.vecFloor(x);
  
  assertEquals(y.length,x.length);
  assertEquals(y[0],1);
  assertEquals(y[1],2);
  assertEquals(y[2],3);
  assertEquals(y[3],4);
  assertEquals(y[4],-3);
}

/**
 * \brief Tests that the inner product between two vectors is calculated correctly.
 */
@Test
public void testDot(){
  double[]x = {1.,1.,1.};
  double[]y = {1.,2.,3.};
  double inner_product_a = GlcMath.dot(x,y);
  
  assertEquals(inner_product_a,6.);
  
  double[]a = {1.,1.,-1.};
  double[]b = {1.,2.,3.};
  double inner_product_b = GlcMath.dot(a,b);
  
  assertEquals(inner_product_b,0.);
}

/**
 * \brief Tests that the square of the norm of a vector is calculated correctly.
 */
@Test
public void testNormSqr(){
  double[]x = {1.,2.,3.};
  double norm_square_a = GlcMath.normSqr(x);
  
  assertEquals(norm_square_a,14.);
  
  double[]y = {1.,2.,-3.};
  double norm_square_b = GlcMath.normSqr(y);
  
  assertEquals(norm_square_b,14.);
}

/**
 * \brief Tests that the 2-norm of a vector is calculated correctly
 */
@Test
public void testNorm(){
  double[]a = {1.,1.,1.};
  double norm_a = GlcMath.norm2(a);
  assertEquals(norm_a,Math.sqrt(3.));
  
  double[]b = {3.,0.,0.};
  double norm_b = GlcMath.norm2(b);
  assertEquals(norm_b,3.);
}


/**
 * \brief Tests that the linearSpace method samples uniformly across an interval
 */
@Test
public void testLinearSpace() {
  double[]points = GlcMath.linearSpace(0.,1.,5);
  assertEquals(points[0],0.0,1e-8);
  assertEquals(points[1],0.2,1e-8);
  assertEquals(points[2],0.4,1e-8);
  assertEquals(points[3],0.6,1e-8);
  assertEquals(points[4],0.8,1e-8);
}




}
