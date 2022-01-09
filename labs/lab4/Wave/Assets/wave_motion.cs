using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour 
{
	int size 		= 100;
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	float damping 	= 0.996f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool 	tag=true;

	float dt = 0.003f;

	Vector3 	cube_v1 = Vector3.zero;
	Vector3 	cube_v2 = Vector3.zero;

	Vector3 	cube_w1 = Vector3.zero;
	Vector3 	cube_w2 = Vector3.zero;
	
	// raycasting direction	
	Vector3 dir    = new Vector3(0.0f, 1.0f, 0.0f);

	// dA
	float dA = 0.01f;

	float rho_w = 1.0f;
	float rho_b = 0.5f;

	float L, V, m, I_inv;

	Vector3 g = new Vector3(0.0f,-9.8f, 0.0f);

	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X=new Vector3[size*size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}

		GameObject block1 = GameObject.Find("Block");

		// hard code some geometries
		L = 1.0f;
		V = L * L * L;
		m = rho_b * V;
		// I is a constant for a cube
		float I = m * L * L / 6.0f;
		I_inv = 1.0f / I;
	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			Ax[i,j]=0;
			if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
			if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
			if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
			if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
		}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}

	void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{		
		//Step 1:
		//TODO: Compute new_h based on the shallow wave model.
		for (int i=0; i<size; i++) {
			for (int j=0; j<size; j++) {
				new_h[i, j] = h[i, j] + damping * (h[i, j] - old_h[i, j]);

				if (i > 0) 		new_h[i, j] += rate * (h[i-1, j] - h[i, j]);
				if (i < size-1) new_h[i, j] += rate * (h[i+1, j] - h[i, j]);
				if (j > 0) 		new_h[i, j] += rate * (h[i, j-1] - h[i, j]);
				if (j < size-1) new_h[i, j] += rate * (h[i, j+1] - h[i, j]);
			}
		}

		//Step 2: Block->Water coupling
		//TODO: for block 1, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		GameObject block1 = GameObject.Find("Block");
		Bounds bbox1 = block1.GetComponent<Renderer>().bounds;
		Vector3 F1 = Vector3.zero + rho_b * g;
		Vector3 T1 = Vector3.zero;

		Quaternion q1 = block1.transform.rotation;
		Matrix4x4 R1 = Matrix4x4.Rotate(q1);

		//TODO: for block 2, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		GameObject block2 = GameObject.Find("Cube");
		Bounds bbox2 = block2.GetComponent<Renderer>().bounds;
		Vector3 F2 = Vector3.zero + rho_b * g * 1.0f;
		Vector3 T2 = Vector3.zero;

		Quaternion q2 = block2.transform.rotation;
		Matrix4x4 R2 = Matrix4x4.Rotate(q2);

		for (int i=0; i<size; i++) {
			for (int j=0; j<size; j++) {
				vh[i, j] = 0;
				cg_mask[i, j] = !tag;

				float x = i*0.1f-size*0.05f;
				float z = j*0.1f-size*0.05f;
				
				Vector3 p = new Vector3(x, new_h[i, j], z);
				
				Vector3 origin = new Vector3(x,-1.0f, z);
				Ray ray = new Ray(origin, dir);
				RaycastHit hit;

				if (bbox1.Contains(p)) {
					if (block1.GetComponent<Collider>().Raycast(ray, out hit, 10.0f)) {
						low_h[i, j] = hit.distance-1.0f;

						if (low_h[i, j] < new_h[i, j]) {
							cg_mask[i, j] = tag;
							b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;

							// force and torque
							Vector3 f1 = rho_w * g * low_h[i, j] * dA;
							F1 += f1;

							Vector3 r = new Vector3(x, low_h[i, j], z);
							r = r - block1.transform.position;
							T1 += Vector3.Cross(r, f1);
						}
					}
				} else if (bbox2.Contains(p)) {
					if (block2.GetComponent<Collider>().Raycast(ray, out hit, 10.0f)) {
						low_h[i, j] = hit.distance-1.0f;

						if (low_h[i, j] < new_h[i, j]) {
							cg_mask[i, j] = tag;
							b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;

							// force and torque
							Vector3 f2 = rho_w * g * low_h[i, j] * dA;
							F2 += f2;

							Vector3 r = new Vector3(x, low_h[i, j], z);
							r = r - block2.transform.position;
							T2 += Vector3.Cross(r, f2);
						}
					}
				}

			}
		}
		
		Conjugate_Gradient(cg_mask, b, vh, 0, size, 0, size);
	
		//TODO: Diminish vh.
		//TODO: Update new_h by vh.
		for (int i=0; i<size; i++) {
			for (int j=0; j<size; j++) {
				vh[i, j] *= gamma;
			}
		}

		//TODO: Update new_h by vh.
		for (int i=0; i<size; i++) {
			for (int j=0; j<size; j++) {
				if (i > 0) 		new_h[i, j] += rate * (vh[i-1, j] - vh[i, j]);
				if (i < size-1) new_h[i, j] += rate * (vh[i+1, j] - vh[i, j]);
				if (j > 0) 		new_h[i, j] += rate * (vh[i, j-1] - vh[i, j]);
				if (j < size-1) new_h[i, j] += rate * (vh[i, j+1] - vh[i, j]);
			}
		}

		//Step 3
		//TODO: old_h <- h; h <- new_h;
		for (int i=0; i<size; i++) {
			for (int j=0; j<size; j++) {
				old_h[i, j] = h[i, j];
				h[i, j] = new_h[i, j];
			}
		}

		//Step 4: Water->Block coupling.
		//More TODO here.
		// block1 velocity
		cube_v1 += F1 / m * dt;
		block1.transform.position += cube_v1 * dt;
		cube_v1 *= damping;

		// block1 angular velocity
		cube_w1 += T1 * I_inv * dt;
		Quaternion dq1= new Quaternion(0.5f * dt * cube_w1.x, 0.5f * dt * cube_w1.y, 0.5f * dt * cube_w1.z, 0.0f);
		dq1 = dq1 * q1;

		q1 = quaternionAdd(q1, dq1);
		q1 = q1.normalized;
		block1.transform.rotation = q1;

		cube_w1 *= damping;

		// block2 velocity
		cube_v2 += F2 / m * dt;
		block2.transform.position += cube_v2 * dt;
		cube_v2 *= damping;

		// block2 angular velocity
		cube_w2 += T2 * I_inv * dt;
		Quaternion dq2= new Quaternion(0.5f * dt * cube_w2.x, 0.5f * dt * cube_w2.y, 0.5f * dt * cube_w2.z, 0.0f);
		dq2 = dq2 * q2;

		q2 = quaternionAdd(q2, dq2);
		q2 = q2.normalized;
		block2.transform.rotation = q2;

		cube_w2 *= damping;
	}
	

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for (int i=0; i<size; i++) {
			for (int j=0; j<size; j++) {
				h[i, j] = X[i*size+j].y;
			}
		}

		if (Input.GetKeyDown ("r")) 
		{
			//TODO: Add random water.
			int ii = Random.Range(1, size-1);
			int jj = Random.Range(1, size-1);

			float r = Random.Range(0.0f, 1.0f);

			h[ii, jj] += r;

			ii = Random.Range(ii-1, ii+1);
			jj = Random.Range(jj-1, jj+1);

			h[ii, jj] -= r;
		}
	
		for(int l=0; l<8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}

		//TODO: Store h back into X.y and recalculate normal.
		for (int i=0; i<size; i++) {
			for (int j=0; j<size; j++) {
				X[i*size+j].y = h[i, j];
			}
		}

		mesh.vertices = X;
		mesh.RecalculateNormals();
	}

	Quaternion quaternionAdd(Quaternion q1, Quaternion q2) {
		Quaternion q = Quaternion.identity;
		q.x = q1.x + q2.x;
		q.y = q1.y + q2.y;
		q.z = q1.z + q2.z;
		q.w = q1.w + q2.w;

		return q;
	}
}
