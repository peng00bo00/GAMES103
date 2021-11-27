using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;

	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision

	float muN = 0.5f;
	float muT = 1.0f;

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Vector3 x = transform.position;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);

		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		// location avg
		int cnt = 0;
		Vector3 p = Vector3.zero;

		// loop over all the vertices
		Vector3 xi = Vector3.zero;
		Vector3 Rri= Vector3.zero;

		for (int i=0; i<vertices.Length; i++) {
			Rri = R.MultiplyPoint3x4(vertices[i]);
			xi = x + Rri;

			// vertex location check
			if (Vector3.Dot(N, xi - P) < 0) {
				// vertex velocity check
				if (Vector3.Dot(N, v + Vector3.Cross(w, Rri)) < 0) {
					cnt++;
					p += vertices[i];
				}
			}
		}

		if (cnt == 0) return;

		// take average of vertices
		p = p / cnt;

		// vertex velocity decomposition
		Rri = R.MultiplyPoint3x4(p);
		Vector3 vi = v + Vector3.Cross(w, Rri);

		Vector3 vN = Vector3.Project(vi, N);
		Vector3 vT = vi - vN;

		// update vertex velocity
		float a = 1.0f - muT * (1.0f + muN) * vN.magnitude / vT.magnitude;
		a = Mathf.Max(a, 0.0f);

		vN = -muN * vN;
		vT = a * vT;

		Vector3 vi_new = vN + vT;

		// solve for impulse J
		Matrix4x4 K = Matrix4x4.Scale(Vector3.one / mass);

		Matrix4x4 I_inv = R * I_ref.inverse * R.transpose;
		Matrix4x4 Rri_cross = Get_Cross_Matrix(Rri);

		K = matrixSub(K, Rri_cross * I_inv * Rri_cross);
		K[3, 3] = 1.0f;

		Vector3 J = Matrix4MultiplyVector3(K.inverse, vi_new - vi);

		// update body velocities
		v = v + J / mass;
		w = w + Matrix4MultiplyVector3(I_inv, Vector3.Cross(Rri, J));
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		if (!launched) return;

		// Part I: Update velocities
		Vector3 g = new Vector3(0, -9.8f, 0);
		v = v + g * dt;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		// Update linear status
		Vector3 x = transform.position;
		x = x + v * dt;

		// Update angular status
		Quaternion q = transform.rotation;
		Quaternion dq= new Quaternion(0.5f * dt * w.x, 0.5f * dt * w.y, 0.5f * dt * w.z, 0.0f);
		dq = dq * q;

		q = quaternionAdd(q, dq);
		q = q.normalized;

		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;

		// Velocities decay
		v = linear_decay * v;
		w = angular_decay * w;
		
	}

	Quaternion quaternionAdd(Quaternion q1, Quaternion q2) {
		Quaternion q = Quaternion.identity;
		q.x = q1.x + q2.x;
		q.y = q1.y + q2.y;
		q.z = q1.z + q2.z;
		q.w = q1.w + q2.w;

		return q;
	}

	Matrix4x4 matrixSub(Matrix4x4 m1, Matrix4x4 m2) {
		Matrix4x4 m = Matrix4x4.identity;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				m[i, j] = m1[i, j] - m2[i, j];
			}
		}

		return m;
	}

	Vector3 Matrix4MultiplyVector3(Matrix4x4 A, Vector3 x) {
		Vector3 y = Vector3.zero;

		y[0] = A[0, 0] * x[0] + A[0, 1] * x[1] + A[0, 2] * x[2];
		y[1] = A[1, 0] * x[0] + A[1, 1] * x[1] + A[1, 2] * x[2];
		y[2] = A[2, 0] * x[0] + A[2, 1] * x[1] + A[2, 2] * x[2];

		return y;
	}
}
