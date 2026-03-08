// Copyright (c) 2023-2026 Gold87 and other Elastic contributors
// This software can be modified and/or shared under the terms
// defined by the Elastic license:
// https://github.com/Gold872/elastic_dashboard/blob/main/LICENSE

package frc.robot.util;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

/** Utility class for communicating with the Elastic FRC dashboard via NetworkTables. */
public final class Elastic {
    private static final StringTopic notificationTopic =
            NetworkTableInstance.getDefault().getStringTopic("/Elastic/RobotNotifications");
    private static final StringPublisher notificationPublisher =
            notificationTopic.publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
    private static final StringTopic selectedTabTopic =
            NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab");
    private static final StringPublisher selectedTabPublisher =
            selectedTabTopic.publish(PubSubOption.keepDuplicates(true));
    private static final ObjectMapper objectMapper = new ObjectMapper();

    private Elastic() {}

    /** Notification severity levels used by the Elastic dashboard. */
    public enum NotificationLevel {
        INFO,
        WARNING,
        ERROR
    }

    /**
     * Sends a notification to the Elastic dashboard.
     *
     * @param notification the {@link Notification} to send
     */
    public static void sendNotification(Notification notification) {
        try {
            notificationPublisher.set(objectMapper.writeValueAsString(notification));
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
    }

    /**
     * Switches the visible Elastic tab by name.
     *
     * <p>If the name is a number string, Elastic selects the tab at that index.
     *
     * @param tabName name of the tab to display
     */
    public static void selectTab(String tabName) {
        selectedTabPublisher.set(tabName);
    }

    /**
     * Switches the visible Elastic tab by index.
     *
     * @param tabIndex zero-based tab index
     */
    public static void selectTab(int tabIndex) {
        selectTab(Integer.toString(tabIndex));
    }

    /** A notification displayed as a pop-up on the Elastic dashboard. */
    public static class Notification {
        @JsonProperty("level")
        private NotificationLevel level;

        @JsonProperty("title")
        private String title;

        @JsonProperty("description")
        private String description;

        @JsonProperty("displayTime")
        private int displayTimeMillis;

        @JsonProperty("width")
        private double width;

        @JsonProperty("height")
        private double height;

        /** Creates a blank INFO notification. Use the {@code with*()} builder methods to fill it. */
        public Notification() {
            this(NotificationLevel.INFO, "", "");
        }

        /**
         * Creates a notification with all fields specified.
         *
         * @param level severity level
         * @param title title text
         * @param description body text
         * @param displayTimeMillis display duration in milliseconds (0 = no auto-dismiss)
         * @param width notification width
         * @param height notification height (-1 = auto)
         */
        public Notification(
                NotificationLevel level,
                String title,
                String description,
                int displayTimeMillis,
                double width,
                double height) {
            this.level = level;
            this.title = title;
            this.description = description;
            this.displayTimeMillis = displayTimeMillis;
            this.width = width;
            this.height = height;
        }

        /** Creates a notification with default display time (3 s) and dimensions. */
        public Notification(NotificationLevel level, String title, String description) {
            this(level, title, description, 3000, 350, -1);
        }

        /** Creates a notification with a custom display time and default dimensions. */
        public Notification(
                NotificationLevel level, String title, String description, int displayTimeMillis) {
            this(level, title, description, displayTimeMillis, 350, -1);
        }

        public NotificationLevel getLevel() {
            return level;
        }

        public String getTitle() {
            return title;
        }

        public String getDescription() {
            return description;
        }

        public int getDisplayTimeMillis() {
            return displayTimeMillis;
        }

        public double getWidth() {
            return width;
        }

        public double getHeight() {
            return height;
        }

        public Notification withLevel(NotificationLevel level) {
            this.level = level;
            return this;
        }

        public Notification withTitle(String title) {
            this.title = title;
            return this;
        }

        public Notification withDescription(String description) {
            this.description = description;
            return this;
        }

        public Notification withDisplaySeconds(double seconds) {
            this.displayTimeMillis = (int) Math.round(seconds * 1000);
            return this;
        }

        public Notification withDisplayMilliseconds(int displayTimeMillis) {
            this.displayTimeMillis = displayTimeMillis;
            return this;
        }

        public Notification withWidth(double width) {
            this.width = width;
            return this;
        }

        public Notification withHeight(double height) {
            this.height = height;
            return this;
        }

        /** Sets the height to -1 so the dashboard infers it automatically. */
        public Notification withAutomaticHeight() {
            this.height = -1;
            return this;
        }

        /** Disables auto-dismiss (display time = 0). */
        public Notification withNoAutoDismiss() {
            this.displayTimeMillis = 0;
            return this;
        }
    }
}
