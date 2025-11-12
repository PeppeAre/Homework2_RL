#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "std_msgs/msg/empty.hpp" // RIMOSSO (non più necessario)
#include "ros2_kdl_package/action/execute_trajectory.hpp"

// Alias per brevità
using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;

class KdlActionClient : public rclcpp::Node
{
public:
    KdlActionClient() : Node("ros2_kdl_action_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<ExecuteTrajectory>(
            this,
            "execute_trajectory");

        RCLCPP_INFO(this->get_logger(), "Client creato. In attesa del server 'execute_trajectory'...");
    }

    void send_goal()
    {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server non disponibile dopo 10 secondi.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Server trovato. Invio il Goal...");

        auto goal_msg = ExecuteTrajectory::Goal();
        
        // --- MODIFICA CORREZIONE ERRORE (Precedente) ---
        // Impostiamo il nuovo campo 'start'
        goal_msg.start = true;
        // --- FINE MODIFICA ---
        
        auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
        
        // Colleghiamo i callback
        send_goal_options.goal_response_callback =
            std::bind(&KdlActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&KdlActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&KdlActionClient::result_callback, this, std::placeholders::_1);
        
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<ExecuteTrajectory>::SharedPtr client_ptr_;

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal è stato rifiutato dal server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accettato dal server, in attesa del risultato...");
        }
    }

    // --- LA CORREZIONE E' QUI ---
    void feedback_callback(
        rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::SharedPtr,
        const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback) // Era shared_fpt
    {
        RCLCPP_INFO(this->get_logger(), "Feedback ricevuto: Errore corrente = %f", feedback->current_error_norm);
    }
    // --- FINE CORREZIONE ---

    void result_callback(const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "RISULTATO: Successo! Messaggio: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "RISULTATO: Goal annullato (Aborted)");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "RISULTATO: Goal annullato (Canceled)");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "RISULTATO: Sconosciuto");
                break;
        }
        rclcpp::shutdown(); // Chiude il client dopo aver ricevuto il risultato
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<KdlActionClient>();
    
    // Invia il goal
    action_client->send_goal();

    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}
