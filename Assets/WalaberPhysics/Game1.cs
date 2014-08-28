#region Using Statements
using System;
using System.Collections.Generic;
using UnityEngine;
#endregion

namespace JelloPhysics
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class Game1 : MonoBehaviour
    {
        JelloPhysics.World mWorld;

		Vector3 cursorPos = Vector3.zero;
		Body dragBody = null;
		int dragPoint = -1;

        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        void Awake()
        {

            // PHYSICS INIT!
            mWorld = new JelloPhysics.World();

            // static ground object.
			GameObject ground = new GameObject ();
			JelloPhysics.ClosedShape groundShape = new JelloPhysics.ClosedShape ();
            groundShape.begin();
            groundShape.addVertex(new Vector2(-10f, -2f));
            groundShape.addVertex(new Vector2(-10f, 2f));
            groundShape.addVertex(new Vector2(10f, 2f));
            groundShape.addVertex(new Vector2(10f, -2f));
            groundShape.finish();

            // make the body.
            JelloPhysics.Body groundBody = new JelloPhysics.Body();
			groundBody.Setup(mWorld, groundShape, float.PositiveInfinity, new Vector2(0f, -5f), 0f, Vector2.one, false);

        }


        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        void Update()
        {
            // UPDATE the physics!
            for (int i = 0; i < 3; i++)
                mWorld.update(1 / 120f);

            // cursor movement.
			cursorPos = Camera.main.ScreenToWorldPoint(Input.mousePosition);

            // if the user presses the A button, create a new body at the cursor position.
            if (Input.GetKeyDown(KeyCode.A))
            {
				GameObject s = new GameObject ();
				JelloPhysics.ClosedShape shape = new JelloPhysics.ClosedShape ();

                shape.begin();
                shape.addVertex(new Vector2(-1.0f, 0f));
                shape.addVertex(new Vector2(0f, 1.0f));
                shape.addVertex(new Vector2(1.0f, 0f));
                shape.addVertex(new Vector2(0f, -1.0f));
                shape.finish();

				DraggableSpringBody body = new DraggableSpringBody();
				body.Setup(mWorld, shape, 1f, 150, 5,300,15, new Vector2(cursorPos.x, cursorPos.y),((float)UnityEngine.Random.Range(0,360)), Vector2.one);

                body.addInternalSpring(0, 2, 400f, 12f);
                body.addInternalSpring(1, 3, 400f, 12f);
            }

			if (Input.GetKeyDown(KeyCode.B))
			{
				GameObject s = new GameObject ();
				JelloPhysics.ClosedShape shape = new JelloPhysics.ClosedShape ();

				shape.begin();
				for (int i = 0; i < 360; i += 20)
				{
					shape.addVertex(new Vector2((float)Mathf.Cos(((float)-i) * Mathf.Deg2Rad), (float)Mathf.Sin(((float)-i) * Mathf.Deg2Rad)));
				}
				shape.finish();
				
				DraggablePressureBody pb = new DraggablePressureBody();
				pb.Setup(mWorld, shape, 1.0f, 40.0f, 10.0f, 1.0f, 300.0f, 20.0f, cursorPos, 0, Vector2.one);
				pb.addTriangle(0, 10, 9);
				pb.addTriangle(0, 9, 1);
				pb.addTriangle(1, 9, 8);
				pb.addTriangle(1, 8, 2);
				pb.addTriangle(2, 8, 7);
				pb.addTriangle(2, 7, 3);
				pb.addTriangle(3, 7, 6);
				pb.addTriangle(3, 6, 4);
				pb.addTriangle(4, 6, 5);
				pb.addTriangle(17, 10, 0);
				pb.addTriangle(17, 11, 10);
				pb.addTriangle(16, 11, 17);
				pb.addTriangle(16, 12, 11);
				pb.addTriangle(15, 12, 16);
				pb.addTriangle(15, 13, 12);
				pb.addTriangle(14, 12, 15);
				pb.addTriangle(14, 13, 12);
				pb.finalizeTriangles(Color.white);
				
				pb.addInternalSpring(0, 2, 400f, 12f);
				pb.addInternalSpring(1, 3, 400f, 12f);
			}

			// dragging!
			if (Input.GetMouseButton(0))
			{
				if (dragBody != null)
				{
					PointMass pm = dragBody.getPointMass(dragPoint);
					if (dragBody.GetType().Name == "DraggableSpringBody")
						((DraggableSpringBody)dragBody).setDragForce(JelloPhysics.VectorTools.calculateSpringForce(pm.Position, pm.Velocity, cursorPos, Vector2.zero, 0.0f, 100.0f, 10.0f), dragPoint);
					else if (dragBody.GetType().Name == "DraggablePressureBody")
						((DraggablePressureBody)dragBody).setDragForce(JelloPhysics.VectorTools.calculateSpringForce(pm.Position, pm.Velocity, cursorPos, Vector2.zero, 0.0f, 100.0f, 10.0f), dragPoint);
					
				}
			}
			else
			{
				dragBody = null;
				dragPoint = -1;
			}

			if (Input.GetMouseButtonDown(0))
			{
				if (dragBody == null)
				{
					int body;
					mWorld.getClosestPointMass(cursorPos, out body, out dragPoint);
					dragBody = mWorld.getBody(body);
				}
			}

        }
    }
}
