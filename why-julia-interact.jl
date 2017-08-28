const n_states = 6; #number of states
const n_aug= 8; #augmented vector dimension
const lambda_ = 3 - n_aug;

#set weights
weights_ = zeros(2*n_aug+1);
weights_[1] = lambda_ / (lambda_ + n_aug);
weights_[2:end] = 0.5/(lambda_ + n_aug);

immutable vehicle #vehicle instance
    id::Int8
    x::Array{Float64,1}
    y::Array{Float64,1}
    v::Array{Float64,1}
    a::Array{Float64,1}
    d::Array{Float64,1}
    w::Array{Float64,1}
end

function UKF(veh::vehicle, Zk_::Array{Float64,2}, Pk_::Array{Float64,2}, Yk_::Array{Float64,2}, ts, std_ad, std_wd)

#UKF implementation for the vehicle motion model CTRA (number of states - 6 (px, py, v, a, θ, w))

  #Prediction
    Zk_aug = vcat(Zk_, [0. 0.]') #kalman state vector augmentation
    Pk_aug = zeros(n_aug, n_aug)
    Pk_aug[1:n_states, 1:n_states] = Pk_
    Pk_aug[7,7] = std_ad^2;
    Pk_aug[8,8] = std_wd^2;

    #Cholesky decomposition
    L = chol(Hermitian(Pk_aug))'

    #sigma points generation
    Zsig_aug = zeros(n_aug, 2*n_aug+1)
    ΔZk_aug = sqrt(lambda_+n_aug)*L
    Zsig_aug = hcat(Zk_aug, Zk_aug .+ ΔZk_aug, Zk_aug .- ΔZk_aug)

    # sigma points prediction
    Zsig_pred = zeros(n_states, 2*n_aug+1)
    for i in 1:2*n_aug+1
      px, py, v_sig, a_sig, θ, w_sig, nu_ad, nu_wd = Zsig_aug[:,i] #extract sigma states

      if (w_sig>0.001)
        px_p = px + ((a_sig*w_sig*ts+v_sig*w_sig)*sin(θ+w_sig*ts)+a_sig*cos(θ+w_sig*ts)-v_sig*w_sig*sin(θ)-a_sig*cos(θ))/w_sig^2
        py_p = py + (a_sig*sin(θ+w_sig*ts)+(-a_sig*w_sig*ts-v_sig*w_sig)*cos(θ+w_sig*ts)-a_sig*sin(θ)+v_sig*w_sig*cos(θ))/w_sig^2
      else
        px_p = px + ((a_sig*ts^2+2*v_sig*ts)*cos(θ))/2
        py_p = py + ((a_sig*ts^2+2*v_sig*ts)*sin(θ))/2
      end

      v_p = v_sig + a_sig*ts;
      θ_p = θ + w_sig*ts;

      #adding noise
      px_p = px_p + 1/6 * nu_ad * ts^3 * cos(θ)
      py_p = py_p + 1/6 * nu_ad * ts^3 * sin(θ)
      v_p = v_p + 1/2 * nu_ad * ts^2
      a_p = a_sig + nu_ad * ts
      θ_p = θ_p + 1/2 * nu_wd * ts^2
      w_p = w_sig + nu_wd * ts

      Zsig_pred[:,i] = [px_p, py_p, v_p, a_p, θ_p, w_p]

      #state vector mean Prediction
      Zk_ = zeros(n_states) #initialization of kalman state vector
      for i in 1:2*n_aug+1
        Zk_ = Zk_ + weights_[i] .* Zsig_pred[:,i]
      end

      #state covariance matrix mean prediciton
      Pk_ = zeros(n_states, n_states) #initialization of kalman state covariance matrix
      for i in 1:2*n_aug+1
        Zdiff = Zsig_pred[:,i] - Zk_
        Zdiff[5] = atan2(sin(Zdiff[5]), cos(Zdiff[5])) #heading angle normalization
        Pk_ = Pk_ + weights_[i] * Zdiff * Zdiff'
      end

    end

  #Update
    #measurement noise variance
    std_px = 3.
    std_py = 3.
    std_v = 8.5
    std_d = 0.75

    if size(Yk_,1)==2
      Rk_ = Diagonal([std_px^2, std_py^2])
      H_ = [1. 0. 0. 0. 0. 0.; #measurement matrix
            0. 1. 0. 0. 0. 0.]
    elseif size(Yk_,1)==3
      Rk_ = Diagonal([std_px^2, std_py^2, std_v^2])
      H_ = [1. 0. 0. 0. 0. 0.; #measurement matrix
            0. 1. 0. 0. 0. 0.;
            0. 0. 1. 0. 0. 0.]
    elseif size(Yk_,1)==4
      Rk_ = Diagonal([std_px^2, std_py^2, std_v^2, std_d^2])
      H_ = [1. 0. 0. 0. 0. 0.; #measurement matrix
            0. 1. 0. 0. 0. 0.;
            0. 0. 1. 0. 0. 0.;
            0. 0. 0. 0. 1. 0.]
    end

    #measurement correction
    Yk_pred = H_ * Zk_ #predicted measurement
    Ydiff = (Yk_ - Yk_pred)
    S_ = H_ * Pk_ * H_' + Rk_
    Kk_ = Pk_ * H_' * inv(S_)
    Zk_ = Zk_ + Kk_ * Ydiff
    Zk_[5] = atan2(sin(Zk_[5]), cos(Zk_[5])) #heading angle normalization
    Pk_ = (eye(n_states) - Kk_ * H_) * Pk_

    return Zk_, Pk_

end

function push_veh(veh::vehicle, data::Array{Float64,2})
    push!(veh.x, data[1])
    push!(veh.y, data[2])
    push!(veh.v, data[3])
    push!(veh.a, data[4])
    push!(veh.d, data[5])
    push!(veh.w, data[6])    
end

function run_ukf(std_ad, std_wd)
	#Generate reference circle path
	r = 15.
	w0 = 0.75
	v0 = r * w0
	a0 = 0.
	d0 = 0.
	x0 = 0
	y0 = 0.
	refVeh = vehicle(Int8(0), [x0], [y0], [v0], [a0], [d0], [w0])
	realVeh = vehicle(Int8(1), [x0], [y0], [v0], [a0], [d0], [w0])

	Zk_ = [x0 y0 v0 a0 d0 w0]'
	Pk_ = 0.75*eye(n_states)
	ukfVeh = vehicle(Int8(2), [Zk_[1]], [Zk_[2]], [Zk_[3]], [Zk_[4]], [Zk_[5]], [Zk_[6]])

	Δt = 0.05
	t = 0.:Δt:15;

	Jreal=[0.]
	wdreal=[0.]

	for i in 2:size(t,1)

	    xref = refVeh.x[end] + refVeh.v[end] * cos(refVeh.d[end]) * Δt + 1/2 * refVeh.a[end] * cos(refVeh.d[end]) * Δt^2
	    yref = refVeh.y[end] + refVeh.v[end] * sin(refVeh.d[end]) * Δt + 1/2 * refVeh.a[end] * sin(refVeh.d[end]) * Δt^2
	    vref = refVeh.v[end] + refVeh.a[end] * Δt
	    θref = refVeh.d[end] + refVeh.w[end] * Δt
	    θref = atan2(sin(θref), cos(θref))
	    
	    push_veh(refVeh, [xref yref refVeh.v[end] refVeh.a[end] θref refVeh.w[end]])
	    
	    push!(Jreal, randn()*std_ad)
	    push!(wdreal, randn()*std_wd)
	    
	    xreal = realVeh.x[end] + realVeh.v[end] * cos(realVeh.d[end]) * Δt + 1/2 * realVeh.a[end] * cos(realVeh.d[end]) * Δt^2 + 1/6 * Jreal[end] * cos(realVeh.d[end]) * Δt^3
	    
	    yreal = realVeh.y[end] + realVeh.v[end] * sin(realVeh.d[end]) * Δt + 1/2 * realVeh.a[end] * sin(realVeh.d[end]) * Δt^2 + 1/6 * Jreal[end] * sin(realVeh.d[end]) * Δt^3
	    
	    vreal = realVeh.v[end] + realVeh.a[end] * Δt + 1/2 * Jreal[end] * cos(realVeh.d[end]) * Δt^2
	    areal = realVeh.a[end] + Jreal[end] * Δt
	    θreal = realVeh.d[end] + realVeh.w[end] * Δt + + 1/2 * wdreal[end] * Δt^2
	    θreal = atan2(sin(θreal), cos(θreal))
	    wreal = realVeh.w[end] + wdreal[end] * Δt
	    
	    push_veh(realVeh, [xreal yreal vreal areal θreal wreal])
	    
	    xmea = realVeh.x[end] + randn()*3.
	    ymea = realVeh.y[end] + randn()*3.
	    Yk_ = [xmea ymea]'
	    Zk_, Pk_ = UKF(ukfVeh, Zk_, Pk_, Yk_, Δt, std_ad, std_wd)
	    push_veh(ukfVeh, Zk_)
	end
	
	RMSE_v = sqrt(mean((realVeh.v - ukfVeh.v).^2.))
	RMSE_a = sqrt(mean((realVeh.a - ukfVeh.a).^2.))
	RMSE_d = sqrt(mean((realVeh.d - ukfVeh.d).^2.))
	RMSE_w = sqrt(mean((realVeh.w - ukfVeh.w).^2.))
	print("RMSE [v a θ ω]: [$(round(RMSE_v,3)), $(round(RMSE_a,3)), $(round(RMSE_d,3)), $(round(RMSE_w,3))]\r")
	flush(STDOUT)

	show_results(realVeh, ukfVeh, t)
end

function show_results(realVeh, ukfVeh, t)

	P1=plot([realVeh.x ukfVeh.x], [realVeh.y ukfVeh.y])

	DATASET=[realVeh.x realVeh.y realVeh.d ukfVeh.x ukfVeh.y ukfVeh.d]
	P2=plot(t, DATASET,line=([:solid :solid :solid], 1),lab=["xref" "yref" "θref" "xukf" "yukf" "θukf"])

	DATASET=[realVeh.v realVeh.a realVeh.w ukfVeh.v ukfVeh.a ukfVeh.w]
	P3=plot(t, DATASET,line=([:solid :solid :solid], 1),lab=["vref" "aref" "wref" "vukf" "aukf" "wukf"])

	return plot(P1, P2, P3, layout=@layout([a{0.65h};b c]))
end
